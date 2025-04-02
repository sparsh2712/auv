import sys
import rclpy
from rclpy.node import Node
from auv_msgs.msg import PwmData
from .can_driver import CanDriver
from std_srvs.srv import Empty
from auv_msgs.srv import TorpedoState, GripperState, MarkerDropperState
import time

class CanMain(Node):
    def __init__(self,port):
        super().__init__("can_main")
        self.pwm_sub = self.create_subscription(PwmData, "/controller/pwm", self.cb_pwm, 5)
        #self.hardkill_CAN=self.create_service(Empty, "/can/hardkill", self.cb_hardkill_CAN)
        #self.softkill_CAN=self.create_service(Empty, "/can/softkill", self.cb_softkill_CAN)
        self.unkill_CAN=self.create_service(Empty, "/can/unkill", self.cb_unkill_CAN)
        self.kill_CAN=self.create_service(Empty, "/can/kill", self.cb_kill_CAN)
        self.dvl_on=self.create_service(Empty, "/dvl_a50/dvl_on", self.cb_dvl_on)
        self.dvl_off=self.create_service(Empty, "dvl_a50/dvl_off", self.cb_dvl_off)
        self.torpedo_state=self.create_service(TorpedoState, "/actuators/torpedo", self.cb_torpedo)
        self.gripper_state=self.create_service(GripperState, "/actuators/gripper", self.cb_gripper)
        self.marker_dropper_state=self.create_service(MarkerDropperState, "/actuators/marker_dropper", self.cb_marker_dropper)
        self.basler_on=self.create_service(Empty, "/can/basler_on", self.cb_basler_on)
        self.basler_off=self.create_service(Empty, "/can/basler_off", self.cb_basler_off)
        self.can_driver=CanDriver(port=port)

        self.kill_CAN_flag=False
        self.actuator_msg_flag=False

        self.torpedo_bits="00"
        self.marker_dropper_bits="00"
        self.gripper_bits="00"

    def cb_dvl_on(self, req,res):
        self.can_driver.dvl_on()
        return res
    
    def cb_dvl_off(self, req, res):
        self.can_driver.dvl_off()
        return res
    
    def cb_basler_on(self,req, res):
        self.can_driver.basler_on()
        return res
    
    def cb_basler_off(self,req,res):
        self.can_driver.basler_off()
        return res
    
    def cb_softkill_CAN(self, req):
        self.kill_CAN_flag=True
        time.sleep(0.2)
        self.can_driver.can_kill()
        print("done softkill")
        return
    
    def cb_hardkill_CAN(self, req):
        self.kill_CAN_flag=True
        time.sleep(0.2)
        self.can_driver.can_kill()
        self.can_driver.close()
        print("done hardkill")
        return
    
    def cb_unkill_CAN(self, req,res):
        self.kill_CAN_flag=False
        self.can_driver.can_unkill()
        self.get_logger().info("done unkill")
        return res

    def cb_kill_CAN(self, req, res):
        self.kill_CAN_flag=False
        self.can_driver.can_kill()
        self.get_logger().info("done kill")
        return res
    
    def cb_pwm(self, msg):
        pwm_msg={}
        pwm_msg["surge_top"]=msg.surge_top
        pwm_msg["surge_left"]=msg.surge_left
        pwm_msg["surge_right"]=msg.surge_right
        pwm_msg["sway_back"]=msg.sway_back
        pwm_msg["sway_front"]=msg.sway_front
        pwm_msg["heave_back"]=msg.heave_back
        pwm_msg["heave_left"]=msg.heave_left
        pwm_msg["heave_right"]=msg.heave_right
        if not self.kill_CAN_flag:
            self.can_driver.send_pwm_message(pwm_msg)
        if self.actuator_msg_flag:
            time.sleep(0.2)
            self.can_driver.send_actuator_message(
                self.torpedo_bits, self.marker_dropper_bits, self.gripper_bits)
            self.actuator_msg_flag=False

    def cb_torpedo(self, req):
        if len(req.torpedo_bits)==2:
            self.torpedo_bits=req.torpedo_bits
            self.actuator_msg_flag=True
        return
    
    def cb_marker_dropper(self, req):
        if len(req.marker_dropper_bits)==2:
            self.marker_dropper_bits=req.marker_dropper_bits
            self.actuator_msg_flag=True
        return
    
    def cb_gripper(self, req):
        if len(req.gripper_bits)==2:
            self.gripper_bits=req.gripper_bits
            self.actuator_msg_flag=True
        return
    
def main(args=None):
    if len(sys.argv)!=2:
        print("Using port /dev/ttyUSB0: give the port as a command line argument if required")
        #port="/dev/ttyUSB1"
    #else:
    port=sys.argv[1]
    rclpy.init(args=args)
    can_main=CanMain(port)
    rclpy.spin(can_main)
    can_main.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
