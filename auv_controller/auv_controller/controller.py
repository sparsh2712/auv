import rclpy 
from rclpy.node import Node 

import numpy as np 
import os
from scipy.spatial.transform import Rotation 
from .allocator import Allocator
from .pid_control import PIDControl
from auv_msgs.msg import ThrusterForces
from auv_msgs.msg import PwmData
from auv_msgs.msg import Pose
from auv_msgs.msg import AuvState 
from auv_msgs.msg import GlobalForces
from .get_constants import get_params
from .get_constants import get_control_law_params

class Controller(Node): 
    def __init__(self): 
        super().__init__('controller')
        self.pose=np.zeros(6, dtype=float)
        self.setpoint=np.zeros(6, dtype=float)
        
        (num_thrust, com_pose, thrust_pose, thrust_orien, power_factors, backward_ratios, max_front_force, max_back_force, max_pwm, min_pwm, zero_pwm) = get_params()

        (Kp, Ki, Kd, maxBack, maxFront) = get_control_law_params()

        self.allocator=Allocator(
            num_thrust,
            com_pose,
            thrust_pose,
            thrust_orien,
            power_factors=power_factors,
            backward_ratios=backward_ratios,
            max_front_force=max_front_force,
            max_back_force=max_back_force,
            max_pwm=max_pwm,
            min_pwm=min_pwm,
            zero_pwm=zero_pwm,
        )

        self.control_law = PIDControl(
            Kp=Kp, Ki=Ki, Kd=Kd,
            maxBack=maxBack, maxFront=maxFront
        )

        self.global_force_pub = self.create_publisher(GlobalForces, '/controller/global_forces', 1)
        self.thrust_force_pub = self.create_publisher(ThrusterForces, '/controller/thruster_forces', 1)
        self.thrust_pwm_pub = self.create_publisher(PwmData, '/controller/pwm', 1)
        self.setpointSub = self.create_subscription(Pose, '/controller/setpoint', self.cbSetpoint,10)
        self.localizationSub = self.create_subscription(AuvState, '/localization/pose', self.cbPose,10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)
        
    def cbSetpoint(self, msg):
        self.setpoint[0]=msg.position.x
        self.setpoint[1]=msg.position.y
        self.setpoint[2]=msg.position.z
        self.setpoint[3]=msg.orientation.roll
        self.setpoint[4]=msg.orientation.pitch
        self.setpoint[5]=msg.orientation.yaw
        return 

    def cbPose(self, msg):
        self.pose[0]=msg.position.x
        self.pose[1]=msg.position.y
        self.pose[2]=msg.position.z
        self.pose[3]=msg.orientation.roll
        self.pose[4]=msg.orientation.pitch
        self.pose[5]=msg.orientation.yaw

    def publish_thruster_forces(self, thrust_forces):
        thrust_msg=ThrusterForces()
        thrust_msg.surge_top=thrust_forces[0]
        thrust_msg.surge_left=thrust_forces[1]
        thrust_msg.surge_right=thrust_forces[2]
        thrust_msg.sway_back=thrust_forces[3]
        thrust_msg.sway_front=thrust_forces[4]
        thrust_msg.heave_back=thrust_forces[5]
        thrust_msg.heave_left=thrust_forces[6]
        thrust_msg.heave_right=thrust_forces[7]
        self.thrust_force_pub.publish(thrust_msg)
        return

    def publish_thruster_pwms(self, thrust_pwms):
        thrust_msg=PwmData()
        thrust_msg.surge_front_left=thrust_pwms[0]
        thrust_msg.surge_front_right=thrust_pwms[1]
        thrust_msg.surge_back_left=thrust_pwms[2]
        thrust_msg.surge_back_right=thrust_pwms[3]
        thrust_msg.heave_front_left=thrust_pwms[4]
        thrust_msg.heave_front_right=thrust_pwms[5]
        thrust_msg.heave_back_left=thrust_pwms[6]
        thrust_msg.heave_back_right=thrust_pwms[7]
        self.thrust_pwm_pub.publish(thrust_msg)
        return

    def publish_global_forces(self, global_forces):
        msg=GlobalForces()
        msg.force.x=global_forces[0]
        msg.force.y=global_forces[1]
        msg.force.z=global_forces[2]
        msg.torque.roll=global_forces[3]
        msg.torque.pitch=global_forces[4]
        msg.torque.yaw=global_forces[5]
        self.global_force_pub.publish(msg)

    def run(self):
        # global_buoyancy = 4.3
        self.control_law.compute_forces(self.setpoint, self.pose)

        global_forces=self.control_law.output
        # global_forces[2] += global_buoyancy
        self.publish_global_forces(global_forces)

        thrust_forces=self.allocator.force_to_thrust(global_forces)
        self.publish_thruster_forces(thrust_forces)

        thrust_pwms=self.allocator.thrust_to_pwm(thrust_forces)
        self.publish_thruster_pwms(thrust_pwms)

def main(args=None):
    rclpy.init(args=args)
    controller=Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
