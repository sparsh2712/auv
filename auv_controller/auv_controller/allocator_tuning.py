import rclpy 
from rclpy.node import Node
import time
from .allocator import Allocator
from .get_constants import get_params
from auv_msgs.msg import ThrusterForces
from auv_msgs.msg import PwmData
from auv_msgs.msg import Pose
from .allocator_tuning_constants import Forces


class Allocator_tuning(Node):
    def __init__(self):
        super().__init__("allocator_tuning")

        self.publisher_thruster= self.create_publisher(PwmData, "/controller/pwm", 1)

    def publish_thruster_pwm(self, thrust_pwms):
        thrust_msg = PwmData()
        thrust_msg.surge_top= thrust_pwms[0]
        thrust_msg.surge_left = thrust_pwms[1]
        thrust_msg.surge_right= thrust_pwms[2]
        thrust_msg.sway_back= thrust_pwms[3]
        thrust_msg.sway_front = thrust_pwms[4]
        thrust_msg.heave_back = thrust_pwms[5]
        thrust_msg.heave_left = thrust_pwms[6]
        thrust_msg.heave_right = thrust_pwms[7]
        self.get_logger().info("su_t %d " %thrust_pwms[0])
        self.get_logger().info("su_l %d " %thrust_pwms[1])
        self.get_logger().info("su_r %d " %thrust_pwms[2])
        self.get_logger().info("sw_b %d " %thrust_pwms[3])
        self.get_logger().info("sw_f %d " %thrust_pwms[4])
        self.get_logger().info("hv_b %d " %thrust_pwms[5])
        self.get_logger().info("hv_l %d " %thrust_pwms[6])
        self.get_logger().info("hv_r %d " %thrust_pwms[7])

        self.publisher_thruster.publish(thrust_msg)
        return

    def run(self):
        force_constants = Forces()
        global_forces = force_constants.getGlobalForces()
        (num_thrust, com_pose, thrust_pose, thrust_orien, power_factors, backward_ratios, max_front_force, max_back_force, max_pwm, min_pwm, zero_pwm) = get_params()
        allocator=Allocator(
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
        thrust_forces=allocator.force_to_thrust(global_forces)
        #allocator.publish_thruster_forces(thrust_forces)
        thrust_pwms= allocator.thrust_to_pwm(thrust_forces)
        #thrust_pwms=[1584, 1590, 1600, 1440, 1456, 1540, 1468, 1540]
        #thrust_pwms = [1530, 1530, 1530, 1530, 1530, 1530, 1530, 1530]
        #self.get_logger().info("Running the thrusters")
        #time.sleep(5)
        self.publish_thruster_pwm(thrust_pwms)
        #self.get_logger().info(thrust_pwms)
        time.sleep(5)
        #self.get_logger().info("Shutting the thrusters off")
        self.publish_thruster_pwm([1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])


def main(args=None):
     rclpy.init(args=args)
     allocator_tuning = Allocator_tuning()
     allocator_tuning.run()
     rclpy.spin(allocator_tuning)
     allocator_tuning.destroy_node()
     rclpy.shutdown()



