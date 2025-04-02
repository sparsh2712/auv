import sys

from action_msgs.msg import GoalStatus
from auv_msgs.action import Interrupt

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import numpy as np

from auv_navigator.constants import NavigatorParams
from auv_msgs.msg import AuvState
from auv_msgs.msg import Pose
from auv_msgs.msg import NavStatus
from auv_msgs.srv import SetpointService
from auv_msgs.srv import ScanService
from auv_msgs.srv import AlignService


class NavigatorClient(Node):

    def __init__(self):
        super().__init__('navigator_client')
        self._action_client = ActionClient(self, Interrupt, 'navigator')

        navigator_params = NavigatorParams()
        self.xy_tolerance, self.z_tolerance, self.roll_tolerance, self.pitch_tolerance, \
        self.yaw_tolerance, self.navigate_time, self.scan_wait_time, self.scans = navigator_params.get_navigator_params()

        self.current_service_type = ''
        self.current_service_id = -1

        self.pose = Pose()
        self.setpoint = Pose()
        self.scan_type = ''
        self.pos_tolerance = self.xy_tolerance
        self.angle_tolerance = self.yaw_tolerance

        self.setpoint_service = self.create_service(SetpointService, "/navigator/setpoint_service", self.cbSetpoint)
        self.scan_service = self.create_service(ScanService, "/navigator/scan_service", self.cbScan)
        self.align_service = self.create_service(AlignService, "/navigator/align_service", self.cbAlign)
        self.nav_status = None

        self.pose_sub = self.create_subscription(AuvState, "/localization/pose", self.cbPose, 10)
        self.status_pub = self.create_publisher(NavStatus, "/navigator/status", 10)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        pass

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.success))
            if format(result.success) is not None:
                self.nav_status = result.success
            else:
                self.nav_status = False

        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def cbPose(self, msg):
        # self.get_logger().info("Pose updated")
        self.pose.position = msg.position
        self.pose.orientation = msg.orientation

    def cbSetpoint(self, req, context):
        self.get_logger().info("New setpoint service call")
        self.setpoint = req.pose
        self.pos_tolerance = req.pos_tolerance
        self.angle_tolerance = req.angle_tolerance
        self.current_service_type = 'setpoint'
        self._action_client.wait_for_server()

        goal_msg = Interrupt.Goal()

        goal_msg.current_service_type = self.current_service_type
        goal_msg.setpoint = self.setpoint
        goal_msg.pos_tolerance = self.pos_tolerance
        goal_msg.angle_tolerance = self.angle_tolerance

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        print(f"{self._send_goal_future.result()} is the result")

        self.current_service_id = self.current_service_id + 1
        status = NavStatus()
        status.service_id = self.current_service_id
        status.status = bool(self.nav_status)

        self.status_pub.publish(status)
        
        response = SetpointService.Response()
        response.service_id = self.current_service_id
        return response
    
    def cbScan(self, req, context):
        self.get_logger().info("New scan service call")
        self.pos_tolerance = self.xy_tolerance
        self.angle_tolerance = self.yaw_tolerance
        self.scan_type = req.scan_type
        self.current_service_type = 'scan'

        self._action_client.wait_for_server()

        goal_msg = Interrupt.Goal()

        goal_msg.current_service_type = self.current_service_type
        goal_msg.scan_type = self.scan_type
        goal_msg.pos_tolerance = self.pos_tolerance
        goal_msg.angle_tolerance = self.angle_tolerance

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.current_service_id = self.current_service_id + 1
        status = NavStatus()
        status.service_id = self.current_service_id
        status.status = False
        print(self.nav_status," = NAV_STATUS")
        self.status_pub.publish(status)
        self.nav_status = 0
        response = ScanService.Response()
        response.service_id = self.current_service_id
        return response
    
    def cbAlign(self, req, context):
        self.get_logger().info("New align service call")
        self.setpoint = req.pose
        self.pos_tolerance = req.pos_tolerance
        self.angle_tolerance = req.angle_tolerance
        self.current_service_type = 'align'

        self._action_client.wait_for_server()

        
        goal_msg = Interrupt.Goal()

        goal_msg.current_service_type = self.current_service_type
        goal_msg.setpoint = self.setpoint
        goal_msg.pos_tolerance = self.pos_tolerance
        goal_msg.angle_tolerance = self.angle_tolerance


        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.current_service_id = self.current_service_id + 1
        status = NavStatus()
        status.service_id = self.current_service_id
        status.status = bool(self.nav_status)
        self.status_pub.publish(status)
        response = AlignService.Response()
        response.service_id = self.current_service_id
        return response

def main(args=None):
    rclpy.init(args=args)

    try:
        action_client = NavigatorClient()

        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()