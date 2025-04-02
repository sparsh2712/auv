#!/usr/bin/python3

import copy
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation

from auv_msgs.msg import DVLOrient
from auv_msgs.msg import Orientation
from auv_msgs.msg import DVLVel
from auv_msgs.msg import PsData
from auv_msgs.msg import AuvState
from sensor_msgs.msg import Imu

class ImuConverion(Node):
    def __init__(self):
        super().__init__('imu')
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
        self.all_data_in = [
            self.get_parameter('imu_status').get_parameter_value().bool_value,
            self.get_parameter('dvl_status').get_parameter_value().bool_value,
            self.get_parameter('dvl_status').get_parameter_value().bool_value,
            self.get_parameter('ps_status').get_parameter_value().bool_value
        ]
        imu_orien = {'roll': self.get_parameter('imu_orientation.roll').get_parameter_value().double_value, 
                    'pitch': self.get_parameter('imu_orientation.pitch').get_parameter_value().double_value,
                    'yaw': self.get_parameter('imu_orientation.yaw').get_parameter_value().double_value
                    }
        self.imu_inv_rot = Rotation.from_euler("XYZ",(-imu_orien['roll'], -imu_orien['pitch'], -imu_orien['yaw']), degrees=True)
        dvl_orien = {'roll': self.get_parameter('dvl_orientation.roll').get_parameter_value().double_value,
                    'pitch': self.get_parameter('dvl_orientation.pitch').get_parameter_value().double_value,
                    'yaw': self.get_parameter('dvl_orientation.yaw').get_parameter_value().double_value
                    }
        # Orientation according to origin
        self.origin_rot = Rotation.identity()
        # Orientation according to IMU w.r.t origin
        self.current_imu_rot = Rotation.identity()
        self.imu_data = Imu()
        self.imu_degree = Orientation()

        default_orien = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.imu_inv_rot = Rotation.from_euler("ZYX", (imu_orien['yaw'], imu_orien['pitch'], imu_orien['roll']), degrees=True).inv()
        self.imu_sub = self.create_subscription(Imu, "/imu/data",  self.cbImu, 10)
        self.irpy = [0,0,0]
        self.belief_pub = self.create_publisher( Orientation, "/imu/degree",1)
        self.create_timer(0.5, self.run)


    def cbImu(self, msg):
        quat = msg.orientation
        origin_inv_rot = self.origin_rot.inv()
        imu_local_reading_rot = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        imu_global_reading_rot = imu_local_reading_rot*self.imu_inv_rot
        self.current_imu_rot = imu_global_reading_rot*origin_inv_rot
       # self.first_data_flag[0] = True
        self.imu_data = msg

    def compute(self):
        imu_data = copy.deepcopy(self.imu_data)
        yaw, pitch, roll = self.current_imu_rot.as_euler("ZYX", degrees=True)
        self.imu_degree.roll = roll
        self.imu_degree.yaw = yaw
        self.imu_degree.pitch = pitch

    def run(self):
        print('in imu')
        self.compute()
        self.belief_pub.publish(self.imu_degree)

def main(args = None):
    rclpy.init(args=args)
    degree = ImuConverion()
    try:
        rclpy.spin(degree)
    
    finally:
        degree.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()


