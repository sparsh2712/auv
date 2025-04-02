import copy
import asyncio
import rclpy
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from auv_msgs.msg import AuvState
from auv_msgs.msg import Pose
from auv_msgs.msg import NavStatus
from auv_msgs.srv import SetpointService
from auv_msgs.srv import ScanService
from auv_msgs.srv import AlignService
from rclpy.action import ActionServer
from auv_msgs.action import Setpoint, Scan, Align
# from constants import get_navigator_params
from auv_navigator.constants import NavigatorParams

class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        navigator_params = NavigatorParams()
        self.xy_tolerance, self.z_tolerance, self.roll_tolerance, self.pitch_tolerance, \
        self.yaw_tolerance, self.navigate_time, self.scan_wait_time, self.scans = navigator_params.get_navigator_params()
        self.pose = Pose()
        self.setpoint = Pose()
        self.scan_type = ''
        self.pos_tolerance = self.xy_tolerance
        self.angle_tolerance = self.yaw_tolerance

        self.current_service_type = ''
        self.interrupt_received = False
        self.current_service_id = -1

        self.setpoint_action_server = ActionServer(self, Setpoint, '/navigator/setpoint_action', self.execute_setpoint_action)
        self.scan_action_server = ActionServer(self, Scan, '/navigator/scan_action', self.execute_scan_action)
        self.align_action_server = ActionServer(self, Align, '/navigator/align_action', self.execute_align_action)


        self.setpoint_service = self.create_service(SetpointService, "/navigator/setpoint_service", self.cbSetpoint)
        self.scan_service = self.create_service(ScanService, "/navigator/scan_service", self.cbScan)
        self.align_service = self.create_service(AlignService, "/navigator/align_service", self.cbAlign)

        self.pose_sub = self.create_subscription(AuvState, "/localization/pose", self.cbPose, 10)

        self.controller_pub = self.create_publisher(Pose, "/controller/setpoint", 1)
        self.status_pub = self.create_publisher(NavStatus, "/navigator/status", 10)

        time.sleep(0.1)

    def cbPose(self, msg):
        self.get_logger().info("Pose updated")
        self.pose.position = msg.position
        self.pose.orientation = msg.orientation
    
    def cbSetpoint(self, req, res):
        self.get_logger().info("New setpoint service call")
        self.setpoint = req.pose
        self.pos_tolerance = req.pos_tolerance
        self.angle_tolerance = req.angle_tolerance
        self.current_service_type = 'setpoint'
        self.interrupt_received = True
        self.current_service_id += 1
        res.service_id = self.current_service_id

        setpoint_achieved = self.depthFirstPlan(self.setpoint)
        if setpoint_achieved:
            self.current_service_type = ''
            status = NavStatus()
            status.service_id = self.current_service_id
            status.status = True
            self.status_pub.publish(status)
        return res
    
    def cbScan(self, req, res):
        self.get_logger().info("New scan service call")
        self.pos_tolerance = self.xy_tolerance
        self.angle_tolerance = self.yaw_tolerance
        self.scan_type = req.scan_type
        self.current_service_type = 'scan'
        self.interrupt_received = True
        self.current_service_id += 1
        res.service_id = self.current_service_id

        scan = self.scans.get("{type_of_scan}".format(type_of_scan=self.scan_type), None)
        if scan is None:
            self.get_logger().warn("Scan not found")
            self.current_service_type = ''
            status = NavStatus()
            status.service_id = self.current_service_id
            status.status = True
            self.status_pub.publish(status)
            return res
        setpoint = Pose()

        for rel_sp in scan["relative_setpoints_list"]:
            pose = copy.deepcopy(self.pose)
            curr_rot = Rotation.from_euler("ZYX", (pose.orientation.yaw, pose.orientation.pitch, pose.orientation.roll), degrees=True)
            setpoint.position.x, setpoint.position.y, setpoint.position.z = np.matmul(curr_rot.as_matrix(), np.array(rel_sp[:3])) + \
                np.array([pose.position.x, pose.position.y, pose.position.z])
            rel_rot = Rotation.from_euler("ZYX", (rel_sp[5], rel_sp[4], rel_sp[3]), degrees=True)
            setpoint.orientation.yaw, setpoint.orientation.pitch, setpoint.orientation.roll = (curr_rot * rel_rot).as_euler("ZYX", degrees=True)
            self.controller_pub.publish(setpoint)
            while self.setpoint_not_reached(setpoint):
                if self.interrupt_received:
                    self.interrupt_received = False
                    return res
                time.sleep(0.25)
            time.sleep(self.scan_wait_time)

        self.current_service_type = ''
        status = NavStatus()
        status.service_id = self.current_service_id
        status.status = True
        self.status_pub.publish(status)
        return res
    
    def cbAlign(self, req, res):
        self.get_logger().info("New align service call")
        self.setpoint = req.pose
        self.pos_tolerance = req.pos_tolerance
        self.angle_tolerance = req.angle_tolerance
        self.current_service_type = 'align'
        self.interrupt_received = True
        self.current_service_id += 1
        res.service_id = self.current_service_id

        setpoint = Pose()
        setpoint.position = self.setpoint.position
        setpoint.orientation = self.setpoint.orientation
        self.controller_pub.publish(setpoint)

        while self.setpoint_not_reached(setpoint):
            if self.interrupt_received:
                self.interrupt_received = False
                return res
            time.sleep(0.25)
        time.sleep(self.navigate_time)

        self.current_service_type = ''
        status = NavStatus()
        status.service_id = self.current_service_id
        status.status = True
        self.status_pub.publish(status)
        return res
    
    def setpoint_not_reached(self, setpoint):
        pose = copy.deepcopy(self.pose)
        xy_difference = np.array([pose.position.x, pose.position.y]) - \
            np.array([setpoint.position.x, setpoint.position.y])
        return np.linalg.norm(xy_difference) > self.pos_tolerance \
            or np.abs(pose.position.z - setpoint.position.z) > self.z_tolerance \
            or np.abs(pose.orientation.roll - setpoint.orientation.roll) > self.roll_tolerance \
            or np.abs(pose.orientation.pitch - setpoint.orientation.pitch) > self.pitch_tolerance \
            or np.abs(pose.orientation.yaw - setpoint.orientation.yaw) > self.angle_tolerance
    
    def run(self):
        self.get_logger().info('nav running')
        if self.interrupt_received:
            self.interrupt_received = False  # Reset the flag
            return
        self.interrupt_received = False

        if self.current_service_type == '':
            time.sleep(0.25)
            return
        
        # if self.current_service_type == 'scan':
        #     scan = self.scans.get("{type_of_scan}".format(type_of_scan=self.scan_type), None)
        #     if scan is None:
        #         self.get_logger().warn("Scan not found")
        #         self.current_service_type = ''
        #         status = NavStatus()
        #         status.service_id = self.current_service_id
        #         status.status = True
        #         self.status_pub.publish(status)
        #         return
        #     setpoint = Pose()

        #     for rel_sp in scan["relative_setpoints_list"]:
        #         pose = copy.deepcopy(self.pose)
        #         curr_rot = Rotation.from_euler("ZYX", (pose.orientation.yaw, pose.orientation.pitch, pose.orientation.roll), degrees=True)
        #         setpoint.position.x, setpoint.position.y, setpoint.position.z = np.matmul(curr_rot.as_matrix(), np.array(rel_sp[:3])) + \
        #             np.array([pose.position.x, pose.position.y, pose.position.z])
        #         rel_rot = Rotation.from_euler("ZYX", (rel_sp[5], rel_sp[4], rel_sp[3]), degrees=True)
        #         setpoint.orientation.yaw, setpoint.orientation.pitch, setpoint.orientation.roll = (curr_rot * rel_rot).as_euler("ZYX", degrees=True)
        #         self.controller_pub.publish(setpoint)
        #         while self.setpoint_not_reached(setpoint):
        #             if self.interrupt_received:
        #                 self.interrupt_received = False
        #                 return
        #             time.sleep(0.25)
        #         time.sleep(self.scan_wait_time)

        #     self.current_service_type = ''
        #     status = NavStatus()
        #     status.service_id = self.current_service_id
        #     status.status = True
        #     self.status_pub.publish(status)
        #     return

        # if self.current_service_type == 'align':
        #     setpoint = Pose()
        #     setpoint.position = self.setpoint.position
        #     setpoint.orientation = self.setpoint.orientation
        #     self.controller_pub.publish(setpoint)

        #     while self.setpoint_not_reached(setpoint):
        #         if self.interrupt_received:
        #             self.interrupt_received = False
        #             return
        #         time.sleep(0.25)
        #     time.sleep(self.navigate_time)

        #     self.current_service_type = ''
        #     status = NavStatus()
        #     status.service_id = self.current_service_id
        #     status.status = True
        #     self.status_pub.publish(status)
        #     return

        # if self.current_service_type == 'setpoint':
        #     setpoint_achieved = self.depthFirstPlan(self.setpoint)
        #     if setpoint_achieved:
        #         self.current_service_type = ''
        #         status = NavStatus()
        #         status.service_id = self.current_service_id
        #         status.status = True
        #         self.status_pub.publish(status)
        #     return

    def depthFirstPlan(self, req):
        setpoint = Pose()
        pose = copy.deepcopy(self.pose)
        setpoint.position.x = pose.position.x
        setpoint.position.y = pose.position.y
        setpoint.position.z = req.position.z
        setpoint.orientation.roll = pose.orientation.roll
        setpoint.orientation.pitch = pose.orientation.pitch
        setpoint.orientation.yaw = pose.orientation.yaw

        self.controller_pub.publish(setpoint)
        while self.setpoint_not_reached(setpoint):
            if self.interrupt_received:
                self.interrupt_received = False
                self.get_logger().info("Interrupted")
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        self.get_logger().info("Depth reached")

        pose = copy.deepcopy(self.pose)
        x = req.position.x - pose.position.x
        y = req.position.y - pose.position.y
        if x > 0.0:
            setpoint.orientation.yaw = np.arctan(y/x) * 180/np.pi
        elif x < 0.0 and y > 0.0:
            setpoint.orientation.yaw = 180.0 + np.arctan(y/x) * 180/np.pi
        elif x < 0.0 and y < 0.0:
            setpoint.orientation.yaw = -180.0 + np.arctan(y/x) * 180/np.pi
        elif y == 0:
            setpoint.orientation.yaw = 0.0 if x >= 0.0 else 180.0
        elif x == 0:
            setpoint.orientation.yaw = 90.0 if (y > 0) else -90.0

        self.controller_pub.publish(setpoint)
        while self.setpoint_not_reached(setpoint):
            if self.interrupt_received:
                self.interrupt_received = False
                self.get_logger().info("Interrupted")
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        self.get_logger().info("Yaw set")
        
        setpoint.position.x = req.position.x
        setpoint.position.y = req.position.y

        self.controller_pub.publish(setpoint)
        while self.setpoint_not_reached(setpoint):
            if self.interrupt_received:
                self.interrupt_received = False
                self.get_logger().info("Interrupted")
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        self.get_logger().info("Position achieved")
        
        setpoint.orientation.roll = req.orientation.roll
        setpoint.orientation.pitch = req.orientation.pitch
        setpoint.orientation.yaw = req.orientation.yaw

        self.controller_pub.publish(setpoint)
        while self.setpoint_not_reached(setpoint):
            if self.interrupt_received:
                self.interrupt_received = False
                self.get_logger().info("Interrupted")
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        self.get_logger().info("Orientation achieved")

        return True

    async def execute_setpoint_action(self, goal_handle):
        self.get_logger().info("Executing setpoint action...")
        self.setpoint = goal_handle.request.pose
        self.pos_tolerance = goal_handle.request.pos_tolerance
        self.angle_tolerance = goal_handle.request.angle_tolerance

        try:
            # Perform depthFirstPlan with interrupt handling
            setpoint_achieved = self.depthFirstPlan(self.setpoint)
            if setpoint_achieved:
                goal_handle.succeed()
                return Setpoint.Result(success=True)
            else:
                goal_handle.abort()
                return Setpoint.Result(success=False)
        except Exception as e:
            self.get_logger().error(f"Error in setpoint action: {e}")
            goal_handle.abort()
            return Setpoint.Result(success=False)

    async def execute_scan_action(self, goal_handle):
        self.get_logger().info(f"Executing scan action for type: {goal_handle.request.scan_type}")
        try:
            scan = self.scans.get("{type_of_scan}".format(type_of_scan=self.scan_type), None)
            if scan is None:
                self.get_logger().warn("Scan not found")
                goal_handle.abort()
                return Scan.Result(success=False)

            setpoint = Pose()
            for rel_sp in scan["relative_setpoints_list"]:
                pose = copy.deepcopy(self.pose)
                curr_rot = Rotation.from_euler("ZYX", (pose.orientation.yaw, pose.orientation.pitch, pose.orientation.roll), degrees=True)
                setpoint.position.x, setpoint.position.y, setpoint.position.z = np.matmul(curr_rot.as_matrix(), np.array(rel_sp[:3])) + \
                    np.array([pose.position.x, pose.position.y, pose.position.z])
                rel_rot = Rotation.from_euler("ZYX", (rel_sp[5], rel_sp[4], rel_sp[3]), degrees=True)
                setpoint.orientation.yaw, setpoint.orientation.pitch, setpoint.orientation.roll = (curr_rot * rel_rot).as_euler("ZYX", degrees=True)
                self.controller_pub.publish(setpoint)

                while self.setpoint_not_reached(setpoint):
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info("Scan action interrupted")
                        goal_handle.canceled()
                        return Scan.Result(success=False)
                    time.sleep(0.25)

                time.sleep(self.scan_wait_time)

            goal_handle.succeed()
            return Scan.Result(success=True)
        except Exception as e:
            self.get_logger().error(f"Error in scan action: {e}")
            goal_handle.abort()
            return Scan.Result(success=False)

    async def execute_align_action(self, goal_handle):
        self.get_logger().info("Executing align action...")
        self.setpoint = goal_handle.request.pose
        self.pos_tolerance = goal_handle.request.pos_tolerance
        self.angle_tolerance = goal_handle.request.angle_tolerance

        try:
            while not self.setpoint_not_reached(self.setpoint):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Align action canceled")
                    goal_handle.canceled()
                    return Align.Result(success=False)
                await asyncio.sleep(0.1)

            goal_handle.succeed()
            return Align.Result(success=True)
        except Exception as e:
            self.get_logger().error(f"Error in align action: {e}")
            goal_handle.abort()
            return Align.Result(success=False)



def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Navigator stopped")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
