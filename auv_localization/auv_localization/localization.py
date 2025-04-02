import rclpy
import copy
from rclpy.node import Node
import numpy as np 
from scipy.spatial.transform import Rotation 

from auv_msgs.msg import DVLOrient
from auv_msgs.msg import DVLVel
from auv_msgs.msg import PsData
from auv_msgs.msg import AuvState
from sensor_msgs.msg import Imu 
from std_msgs.msg import String

from std_srvs.srv import Empty 

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.declare_parameters(
             namespace='',
             parameters=[
                 ('imu_status', rclpy.Parameter.Type.BOOL),
                 ('dvl_status', rclpy.Parameter.Type.BOOL),
                 ('ps_status', rclpy.Parameter.Type.BOOL),
                 ('dvl_vel_integrate', rclpy.Parameter.Type.BOOL),
                 ('imu_orientation.roll', rclpy.Parameter.Type.DOUBLE),
                 ('imu_orientation.pitch', rclpy.Parameter.Type.DOUBLE),
                 ('imu_orientation.yaw', rclpy.Parameter.Type.DOUBLE),
                 ('dvl_orientation.roll', rclpy.Parameter.Type.DOUBLE),
                 ('dvl_orientation.pitch', rclpy.Parameter.Type.DOUBLE),
                 ('dvl_orientation.yaw', rclpy.Parameter.Type.DOUBLE)
                ]
        )
        self.get_logger().info("hello World")
        self.all_data_in = [
            self.get_parameter('imu_status').get_parameter_value().bool_value,
            self.get_parameter('dvl_status').get_parameter_value().bool_value,
            self.get_parameter('dvl_status').get_parameter_value().bool_value,
            self.get_parameter('ps_status').get_parameter_value().bool_value
        ]
        self.imu_status = self.all_data_in[0]
        self.dvl_status = self.all_data_in[1]
        self.dvl_vel_integrate = self.all_data_in[2]
        self.ps_status = self.all_data_in[3]
        #self.imu_status = self.get_parameter('imu_status').get_parameter_value().bool_value
        # self.imu_status = True
        # self.dvl_status = True
        # self.ps_status = True
        # self.dvl_vel_integrate = True
        # self.all_data_in = [self.imu_status, self.dvl_status, self.dvl_status, self.ps_status]
        self.first_data_flag = [True, True, True, True]
        self.first_reset_done=False
        self.origin_dvl=DVLOrient()
        self.origin_imu=Imu()
        self.origin_state=AuvState()
        
        self.imu_data = Imu()
        self.dvl_vel_data = DVLVel()
        self.dvl_orient_data = DVLOrient()
        self.ps_data = PsData()
        self.belief = AuvState()

        default_orien = {'roll': 0,
                         'pitch': 0,
                         'yaw': 0}
        imu_orien = {'roll': self.get_parameter('imu_orientation.roll').get_parameter_value().double_value, 
                    'pitch': self.get_parameter('imu_orientation.pitch').get_parameter_value().double_value,
                    'yaw': self.get_parameter('imu_orientation.yaw').get_parameter_value().double_value
                    }
        self.imu_inv_rot = Rotation.from_euler("XYZ",(-imu_orien['roll'], -imu_orien['pitch'], -imu_orien['yaw']), degrees=True)
        dvl_orien = {'roll': self.get_parameter('dvl_orientation.roll').get_parameter_value().double_value,
                    'pitch': self.get_parameter('dvl_orientation.pitch').get_parameter_value().double_value,
                    'yaw': self.get_parameter('dvl_orientation.yaw').get_parameter_value().double_value
                    }
        self.dvl_inv_rot = Rotation.from_euler('XYZ', (-dvl_orien['roll'], -dvl_orien['pitch'], -dvl_orien['yaw']), degrees=True)
        self.sample_pub = self.create_publisher(String, 'test_topic2', 10)
        self.belief_pub = self.create_publisher(AuvState, '/localization/pose', 1)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.cbImu,10)
        self.dvl_vel_sub = self.create_subscription(DVLVel, '/dvl/velData', self.cbDvlVel,10)
        self.dvl_orient_sub = self.create_subscription(DVLOrient,'/dvl/orientData',self.cbDvlOrient,10)
        self.ps_sub = self.create_subscription(PsData, '/ps/data', self.cbPs,10)
        self.sample_sub = self.create_subscription(String,'test_topic', self.cbSample_sub,10)
        self.previous_dvl_time = 0
        self.integrated_position = np.zeros(3, dtype=float)
        self.reset_service = self.create_service(Empty, '/localization/reset_service',self.cb_reset)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.run)
        self.i = 0


    def cbSample_sub(self,msg):
        self.get_logger().info(f'I heard "%s"' % msg.data)

    def cbImu(self,msg):
        self.first_data_flag[0] = True
        self.imu_data = msg 
        
    def cbDvlVel(self,msg):
        if self.first_data_flag[0]==True and self.first_data_flag[1]==True:
            rel_quat = self.imu_data.orientation
            reading = Rotation.from_quat([rel_quat.x, rel_quat.y, rel_quat.z, rel_quat.w])
            global_rot = self.imu_inv_rot*reading
            local_vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
            global_vel = np.matmul(global_rot.as_matrix(), local_vel)
            time_diff = msg.time - self.previous_dvl_time
            self.integrated_position += global_vel*time_diff
        self.previous_dvl_time = msg.time
        self.first_data_flag[1] = True 
        self.dvl_vel_data = msg

    def cbDvlOrient(self, msg):
        self.first_data_flag[2] = True
        self.dvl_orient_data = msg

    def cbPs(self, msg):
        self.first_data_flag[3] = True
        self.ps_data = msg

    def cb_reset(self, req):
        self.origin_state.position.x=self.belief.position.x
        self.origin_state.position.y=self.belief.position.y
        self.origin_state.orientation.yaw = self.belief.orientation.yaw
        #add for z
        return[]

    def computeBelief(self):
        self.belief.header.stamp = self.get_clock().now().to_msg()
        imu_data = copy.deepcopy(self.imu_data)
        dvl_vel_data = copy.deepcopy(self.dvl_vel_data)
        dvl_orient_data = copy.deepcopy(self.dvl_orient_data)
        ps_data = copy.deepcopy(self.ps_data)
        
        #All sensors working
        if(self.imu_status and self.dvl_status and self.ps_status):
            self.belief.position = dvl_orient_data.position
            self.belief.position.z = ps_data.depth
            self.belief.velocity = dvl_vel_data.velocity
            self.belief.acceleration = imu_data.linear_acceleration

            rel_quat = imu_data.orientation
            reading = Rotation.from_quat([rel_quat.x, rel_quat.y, rel_quat.z, rel_quat.w])
            global_orien = (self.imu_inv_rot*reading).as_euler("ZYX", degrees=True)
            self.belief.orientation.roll = global_orien[2]
            self.belief.orientation.pitch = global_orien[1]
            self.belief.orientation.yaw = global_orien[0]

            rel_ang_vel = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
            global_ang_vel = np.matmul(self.imu_inv_rot.as_matrix(), rel_ang_vel)
            self.belief.angular_velocity.x = global_ang_vel[0]
            self.belief.angular_velocity.y = global_ang_vel[1]
            self.belief.angular_velocity.z = global_ang_vel[2]

            self.belief.height = dvl_vel_data.altitude
            self.belief.depth = ps_data.depth

        #Only Imu out 
        if(not self.imu_status and self.dvl_status and self.ps_status):
            self.belief.position = dvl_orient_data.position
            self.belief.position.z = ps_data.depth
            self.belief.velocity = dvl_vel_data.velocity

            # dvl orientation is in degrees
            reading = Rotation.from_euler("ZYX",(dvl_orient_data.yaw, dvl_orient_data.pitch, dvl_orient_data.roll), degrees=True)
            global_orien = (self.dvl_inv_rot*reading).as_euler("ZYX", degrees=True)
            self.belief.orientation.roll = global_orien[2]
            self.belief.orientation.pitch = global_orien[1]
            self.belief.orientation.yaw = global_orien[0]

            self.belief.height = self.dvl_vel_data.altitude
            self.belief.depth = self.ps_data.depth

        #Only Ps Out
        if(self.imu_status and self.dvl_status and (not self.ps_status)):
            self.belief.position = dvl_orient_data.position
            self.belief.velocity = dvl_vel_data.velocity
            self.belief.acceleration = imu_data.linear_acceleration

            rel_quat = imu_data.orientation
            reading = Rotation.from_quat([rel_quat.x, rel_quat.y, rel_quat.z, rel_quat.w])
            global_orien = (self.imu_inv_rot*reading).as_euler("ZYX", degrees=True)
            self.belief.orientation.roll = global_orien[2]
            self.belief.orientation.pitch = global_orien[1]
            self.belief.orientation.yaw = global_orien[0]

            rel_ang_vel = np.array([imu_data.angular_velocity.x,
                imu_data.angular_velocity.y, imu_data.angular_velocity.z])
            global_ang_vel = np.matmul(self.imu_inv_rot.as_matrix(), rel_ang_vel)
            self.belief.angular_velocity.x = global_ang_vel[0]
            self.belief.angular_velocity.y = global_ang_vel[1]
            self.belief.angular_velocity.z = global_ang_vel[2]

            self.belief.height = dvl_vel_data.altitude

        if self.dvl_vel_integrate: 
            self.belief.position.x = self.integrated_position[0]
            self.belief.position.y = self.integrated_position[1]
            self.belief.position.z = self.integrated_position[2]
            if self.ps_status:
                self.belief.position.z = self.ps_data.depth

    def convert_origin_frame(self):
        origin_yaw = self.origin_state.orientation.yaw
        origin_x = self.origin_state.position.x
        origin_y = self.origin_state.position.y

        rel_x = self.belief.position.x - origin_x
        rel_y = self.belief.position.y - origin_y
        #add for z
        self.belief.orientation.yaw = self.belief.orientation.yaw - origin_yaw
        self.belief.position.x = np.cos(origin_yaw) * rel_x + np.sin(origin_yaw) * rel_y
        self.belief.position.y = -np.sin(origin_yaw) * rel_x + np.cos(origin_yaw) * rel_y
        # velocity and acceleration transformtions pending 


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.sample_pub.publish(msg)
        #self.get_logger().info('Publishing')
        self.i +=1

    def run(self):
        if(self.first_data_flag == self.all_data_in):
            self.computeBelief()
            self.convert_origin_frame()
            self.belief.position.x *= 100
            self.belief.position.y *= 100
            self.belief.position.z *= 100/17
            self.belief_pub.publish(self.belief)
            self.belief.position.x /= 100
            self.belief.position.y /= 100
            self.belief.position.z /= 100


def main(args=None):
    rclpy.init(args=args)
    localization = Localization()
    try:
        localization.run() 
        rclpy.spin(localization)
    
    finally:
        localization.destroy_node()
        rclpy.shutdown()

    

if __name__ == '__main__':
    main()
