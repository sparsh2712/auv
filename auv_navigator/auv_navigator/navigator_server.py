import sys
import threading
import numpy as np
import time
import copy

from auv_msgs.action import Interrupt
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scipy.spatial.transform import Rotation

from auv_navigator.constants import NavigatorParams
from auv_msgs.msg import AuvState
from auv_msgs.msg import Pose
from std_msgs.msg import String



class NavigatorServer(Node):
    """Minimal action server that processes one goal at a time."""

    def __init__(self):
        super().__init__('naviagtor_server')

        self.current_service_type = ''

        navigator_params = NavigatorParams()
        self.xy_tolerance, self.z_tolerance, self.roll_tolerance, self.pitch_tolerance, \
        self.yaw_tolerance, self.navigate_time, self.scan_wait_time, self.scans = navigator_params.get_navigator_params()

        self.pose = Pose()
        self.setpoint = Pose()
        self.scan_type = ''
        self.pos_tolerance = self.xy_tolerance
        self.angle_tolerance = self.yaw_tolerance
        self.string_data = str('')

        self.pose_sub = self.create_subscription(AuvState, "/localization/pose", self.cbPose, 10)
        self.feed_sub = self.create_subscription(String, "/feedback/input", self.feed_test, 10)
        

        self.controller_pub = self.create_publisher(Pose, "/controller/setpoint", 1)
        self.timer_callback = self.create_timer(1,self.feedback_timer)

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.feedback_msg = Interrupt.Feedback()
        self._action_server = ActionServer(
            self,
            Interrupt,
            'navigator',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=None)
        
    def feedback_timer(self):
        
        if self._goal_handle is not None:
            self.feedback_msg.setpoint_detect = self.string_data
        # print(self.feedback_msg)
            self._goal_handle.publish_feedback(self.feedback_msg)
        
    def cbPose(self, msg):
        self.get_logger().info("Pose updated")
        self.pose.position = msg.position
        self.pose.orientation = msg.orientation

    def feed_test(self, msg):
        self.string_data = str(msg.data)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
                self._goal_handle = None
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def setpoint_not_reached(self, setpoint):
        pose = copy.deepcopy(self.pose)
        xy_difference = np.array([pose.position.x, pose.position.y]) - \
            np.array([setpoint.position.x, setpoint.position.y])
        return np.linalg.norm(xy_difference) > self.pos_tolerance \
            or np.abs(pose.position.z - setpoint.position.z) > self.z_tolerance \
            or np.abs(pose.orientation.roll - setpoint.orientation.roll) > self.roll_tolerance \
            or np.abs(pose.orientation.pitch - setpoint.orientation.pitch) > self.pitch_tolerance \
            or np.abs(pose.orientation.yaw - setpoint.orientation.yaw) > self.angle_tolerance

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        self.current_service_type = goal_handle.request.current_service_type

        print(self.current_service_type)       

        if self.current_service_type == '':
            goal_handle.succeed()
            result = Interrupt.Result()
            result.success = True
            time.sleep(0.25)
            return result
        
        if(self.current_service_type=='scan'):
            self.scan_type = goal_handle.request.scan_type
            scan = self.scans.get("{type_of_scan}".format(type_of_scan=self.scan_type), None)
            if scan == None:
                print("Scan not found")
                self.current_service_type = ''
                result = Interrupt.Result()
                result.success = True
                if goal_handle.is_active:
                    goal_handle.succeed()
                return result
            setpoint = Pose()

            for rel_sp in scan["relative_setpoints_list"]:
                pose = copy.deepcopy(self.pose)
                curr_rot = Rotation.from_euler("ZYX", (pose.orientation.yaw, pose.orientation.pitch, pose.orientation.roll), degrees=True)
                setpoint.position.x, setpoint.position.y, setpoint.position.z = np.matmul(curr_rot.as_matrix(), np.array(rel_sp[:3])) + \
                    np.array([pose.position.x, pose.position.y, pose.position.z])
                rel_rot = Rotation.from_euler("ZYX", (rel_sp[5], rel_sp[4], rel_sp[3]), degrees=True)
                setpoint.orientation.yaw, setpoint.orientation.pitch, setpoint.orientation.roll = (curr_rot*rel_rot).as_euler("ZYX", degrees=True)
                
                self.controller_pub.publish(setpoint)
                while(self.setpoint_not_reached(setpoint)):
                    result = Interrupt.Result()
                    if not goal_handle.is_active:
                        self.get_logger().info('Goal aborted')
                        result.success = False
                        return result

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.get_logger().info('Goal canceled')
                        result.success = False
                        return result

                    rate = self.create_rate(0.5)
                    rate.sleep()
                rate = self.create_rate(self.scan_wait_time)
                rate.sleep()

            self.current_service_type = ''
            result = Interrupt.Result()
            result.success = True
            if goal_handle.is_active:
                goal_handle.succeed()
            return result
        
        if(self.current_service_type=='align'):
            setpoint = Pose()
            self.setpoint = goal_handle.request.setpoint
            setpoint.position = self.setpoint.position
            setpoint.orientation = self.setpoint.orientation
            self.controller_pub.publish(setpoint)

            while(self.setpoint_not_reached(setpoint)):
                result = Interrupt.Result()
                if not goal_handle.is_active:
                    self.get_logger().info('Goal aborted')
                    result.success = False
                    return result

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    result.success = False
                    return result
                time.sleep(0.25)
            time.sleep(self.navigate_time)

            self.current_service_type = ''
            
            result = Interrupt.Result()
            result.success = True
            if goal_handle.is_active:
                goal_handle.succeed()
            return result

        if(self.current_service_type=='setpoint'):
            print("Depth first plan")
            self.setpoint = goal_handle.request.setpoint
            setpoint_achieved = self.depthFirstPlan(self.setpoint, goal_handle)
            result = Interrupt.Result()
            if setpoint_achieved:
                self.current_service_type = ''
                result.success = True
                if goal_handle.is_active:
                    goal_handle.succeed()
            else:
                result.success = False
            return result
        
        if(self.current_service_type=='scan_align'):
            print("Continuous Align")
            self.setpoint = goal_handle.request.setpoint
            setpoint_achieved = self.ScanAlign(self.setpoint, goal_handle)
            result = Interrupt.Result()
            if setpoint_achieved:
                self.current_service_type = ''
                result.success = True
                if goal_handle.is_active:
                    goal_handle.succeed()
            else:
                result.success = False
            return result

    def depthFirstPlan(self, req, goal_handle):

        setpoint = Pose()
        pose = copy.deepcopy(self.pose)
        setpoint.position.x = pose.position.x
        setpoint.position.y = pose.position.y
        setpoint.position.z = req.position.z
        setpoint.orientation.roll = pose.orientation.roll
        setpoint.orientation.pitch = pose.orientation.pitch
        setpoint.orientation.yaw = pose.orientation.yaw

        self.pos_tolerance = goal_handle.request.pos_tolerance
        self.angle_tolerance = goal_handle.request.angle_tolerance

        self.controller_pub.publish(setpoint)
        while(self.setpoint_not_reached(setpoint)):
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return False

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        print("Depth reached")

        pose = copy.deepcopy(self.pose)
        x = req.position.x - pose.position.x
        y = req.position.y - pose.position.y
        if(x>0):
            setpoint.orientation.yaw = np.arctan(y/x)*180/np.pi
        elif(x<0 and y>0):
            setpoint.orientation.yaw = 180 + np.arctan(y/x)*180/np.pi
        elif(x<0 and y<0):
            setpoint.orientation.yaw = -180 + np.arctan(y/x)*180/np.pi
        elif(y==0):
            setpoint.orientation.yaw = 0.0 if(x>=0) else 180.0
        elif(x==0):
            setpoint.orientation.yaw = 90.0 if (y>0) else -90.0

        self.controller_pub.publish(setpoint)
        while(self.setpoint_not_reached(setpoint)):
            result = Interrupt.Result()
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return False

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        print("Yaw set")
        
        setpoint.position.x = req.position.x
        setpoint.position.y = req.position.y

        self.controller_pub.publish(setpoint)
        while(self.setpoint_not_reached(setpoint)):
            result = Interrupt.Result()
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return False

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        print("Position achieved")
        
        setpoint.orientation.roll = req.orientation.roll
        setpoint.orientation.pitch = req.orientation.pitch
        setpoint.orientation.yaw = req.orientation.yaw

        self.controller_pub.publish(setpoint)
        while(self.setpoint_not_reached(setpoint)):
            result = Interrupt.Result()
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return False

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        print("Orientation achieved")

        return True
    
    def ScanAlign(self,req,goal_handle):
        setpoint = Pose()
        pose = copy.deepcopy(self.pose)
        setpoint.position.x = req.position.x
        setpoint.position.y = req.position.y
        setpoint.position.z = req.position.z
        setpoint.orientation.roll = req.orientation.roll
        setpoint.orientation.pitch = req.orientation.pitch
        setpoint.orientation.yaw = req.orientation.yaw

        self.pos_tolerance = goal_handle.request.pos_tolerance
        self.angle_tolerance = goal_handle.request.angle_tolerance

        self.controller_pub.publish(setpoint)
        while(self.setpoint_not_reached(setpoint)):
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return False

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return False
            time.sleep(0.25)
        time.sleep(self.navigate_time)
        print("Depth reached")

def main(args=None):
    rclpy.init(args=args)

    try:
        action_server = NavigatorServer()

        # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
        executor = MultiThreadedExecutor()
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()