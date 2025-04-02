import rclpy
import copy
from rclpy.node import Node
import numpy as np 
from scipy.spatial.transform import Rotation 

from auv_msgs.msg import DVLOrient
from auv_msgs.msg import DVLVel
from auv_msgs.msg import PsData
from auv_msgs.msg import AuvState
from auv_msgs.msg import Pose
from sensor_msgs.msg import Imu 
from std_msgs.msg import String
import time
from std_srvs.srv import Empty 

class LocalizationDummy(Node):
    def __init__(self):
        super().__init__('localization_dummy')
        self.localization = self.create_publisher(AuvState, '/localization/pose', 10)
        self.setpointSub = self.create_subscription(Pose, '/controller/setpoint', self.cbSetpoint,10)
        self.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.timer = self.create_timer(2.0, self.publish_pose)

    def cbSetpoint(self, msg):
        self.pose[0]=msg.position.x
        self.pose[1]=msg.position.y
        self.pose[2]=msg.position.z
        self.pose[3]=msg.orientation.roll
        self.pose[4]=msg.orientation.pitch
        self.pose[5]=msg.orientation.yaw
        return

    def publish_pose(self):
        msg=AuvState()
        msg.position.x=self.pose[0]
        msg.position.y=self.pose[1]
        msg.position.z=self.pose[2]
        msg.orientation.roll=self.pose[3]
        msg.orientation.pitch=self.pose[4]
        msg.orientation.yaw=self.pose[5]
        print("publishing_data")
        time.sleep(0.5)
        self.localization.publish(msg)
        return 
    
def main(args=None):
    rclpy.init(args=args)
    localization = LocalizationDummy()
    try:
        localization.publish_pose() 
        rclpy.spin(localization)
    
    finally:
        localization.destroy_node()
        rclpy.shutdown()

    

if __name__ == '__main__':
    main()
    



