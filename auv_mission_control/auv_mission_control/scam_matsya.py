import copy
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import asyncio
import time
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from auv_msgs.action import Interrupt
from rclpy.executors import ExternalShutdownException


import rclpy
from rclpy.action import ActionClient

from std_srvs.srv import Empty
from auv_msgs.msg import AuvState, Pose, VisionSetpoints, NavStatus
from auv_msgs.srv import SetpointService
from auv_msgs.srv import AlignService
from auv_msgs.srv import ScanService
from auv_msgs.srv import TorpedoState
from auv_msgs.srv import GripperState
from auv_msgs.srv import MarkerDropperState
from auv_msgs.srv import PingerAcq
from auv_msgs.srv import PwmCap
from auv_msgs.srv import SetDetector
from auv_msgs.msg import CurrentNavId
from rclpy.executors import SingleThreadedExecutor
from auv_navigator.constants import NavigatorParams



class Matsya:#(Node):
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()

        self.logger.info("Initializing Matsya...")
        self._action_client = ActionClient(self.node, Interrupt, 'navigator')
        navigator_params = NavigatorParams()
        self.xy_tolerance, self.z_tolerance, self.roll_tolerance, self.pitch_tolerance, \
        self.yaw_tolerance, self.navigate_time, self.scan_wait_time, self.scans = navigator_params.get_navigator_params()
        self.result = False
        # self.goal_handle = None


        # Initialize services
        try:
            self.nav_setpoint_proxy = self.node.create_client(SetpointService, "/navigator/setpoint_service")
            self.nav_align_proxy = self.node.create_client(AlignService, "/navigator/align_service")
            self.nav_scan_proxy = self.node.create_client(ScanService, "/navigator/scan_service")
            self.vision_toggle_proxy = self.node.create_client(SetDetector, "/vision/set_detector")
            self.acoustics_proxy = self.node.create_client(PingerAcq, "/pinger/process")
            self.basler_off_proxy = self.node.create_client(Empty, "/can/basler_off")
            self.basler_on_proxy = self.node.create_client(Empty, "/can/basler_on")
            self.torpedo_proxy = self.node.create_client(TorpedoState, "/actuators/torpedo")
            self.gripper_proxy = self.node.create_client(GripperState, "/actuators/gripper")
            self.marker_dropper_proxy = self.node.create_client(MarkerDropperState, "/actuators/marker_dropper")
            self.pwm_cap_proxy = self.node.create_client(PwmCap, "/controller/pwm_cap")
        except Exception as e:
            self.logger.error(f"Error initializing service clients: {e}")
            raise

        # Initialize subscribers
        self.nav_status_sub = self.node.create_subscription(NavStatus, "/navigator/status", self.cb_nav, 10)
        self.vision_sub = self.node.create_subscription(VisionSetpoints, "/vision/detection", self.cb_vision, 10)
        self.localization_sub = self.node.create_subscription(AuvState, "/localization/pose", self.cb_pose, 10)
        self.nav_status = self.node.create_publisher(CurrentNavId,"/mission_control/current_service_id", 1)

        #PWM Cap
        req = PwmCap.Request()
        req.max_pwm = 1780
        # self.pwm_cap_proxy(req)
        
        # Internal state
        self.pose = AuvState()
        self.pose_is_valid = False
        self.vision_detections = VisionSetpoints()
        self.current_nav_service_id = 0
        self.in_transit = False
        self.logger.info("[Matsya] Matsya initialized successfully.")
        timer_period = 1
        self.timer = self.node.create_timer(timer_period, self.pose_valid)


    def pose_valid(self):
        if self.pose_is_valid is not True:
            print("waiting for localisation")

        else:
            #destroy timer
            self.timer.destroy()
            print("localisation is now valid")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._goal_handle = goal_handle
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return

        self.node.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        return self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        pass
        # self.node.get_logger().info('Received feedback: {0}'.format(feedback.feedback))


    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Goal succeeded! Result: {0}'.format(result.success))
            self.result = result.success
            if format(result.success) is not None:
                self.nav_status = result.success
            else:
                self.nav_status = False

        else:
            self.node.get_logger().info('Goal failed with status: {0}'.format(status))
            return False

    def cb_pose(self, msg: AuvState):
        self.pose = msg
        self.pose_is_valid = True
        # print("RECEIVED AND WORKING AS EXPECTED")
        print(f"Updated Matsya pose: {msg.position.x} {msg.position.y} {msg.position.z} {msg.orientation.yaw}")
        

    def cb_vision(self, msg: VisionSetpoints):
        self.vision_detections = msg
    
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def cb_nav(self, msg: NavStatus):
        print("Inside a callback from navigator", self.current_nav_service_id)
        print(f"msg service id, {msg.service_id}")
        print(f"msg status, {msg.status}")
        if msg.status and msg.service_id == self.current_nav_service_id:
            print("Callback from navigator")
            self.in_transit = False


    async def toggle_camera(self, camera: str):
        cam_req = SetDetector.Request()
        cam_req.camera = camera
        cam_res = await self.vision_toggle_proxy.call_async(cam_req)
        return cam_res

    def check_vision(self, task_name: str):
        req = Empty.Request()
        self.basler_on_proxy.call_async(req)
        # self.node.get_logger().info('Camera On')
        detections = copy.deepcopy(self.vision_detections.tasks)
        for i in range(len(detections)):
            if detections[i].name != task_name:
                continue

            task_detected = True
            task_location = [detections[i].position.x,
                             detections[i].position.y,
                             detections[i].position.z]
            self.basler_off_proxy.call_async(req)
            return task_detected, task_location
        self.basler_off_proxy.call_async(req)
        return False, [0.0, 0.0, 0.0]

    def reach_task(self, task_location: list, pos_tolerance: float, angle_tolerance: float, timeout: float = -1):
        nav_req = Pose()
        nav_req.position.x = float(task_location['x'])
        nav_req.position.y = float(task_location['y'])
        nav_req.position.z = float((task_location['z']))
        nav_req.orientation.roll = float(task_location['roll'])
        nav_req.orientation.pitch = float(task_location['pitch'])
        nav_req.orientation.yaw = float(task_location['yaw'])
        self.current_service_type = 'setpoint'
        # self.current_service_type = 'scan_align'
        self._action_client.wait_for_server()
        goal_msg = Interrupt.Goal()
        goal_msg.current_service_type = self.current_service_type
        goal_msg.setpoint = nav_req
        goal_msg.pos_tolerance = float(pos_tolerance)
        goal_msg.angle_tolerance = float(angle_tolerance)
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        future = self._send_goal_future.add_done_callback(self.goal_response_callback)

        ctr = 1
        while(1):
            rclpy.spin_once(self.node, timeout_sec = 1)
            time.sleep(0.1)
            if self.result:
                break

        task_reached = self.result
        self.result = False
        print(task_reached, "helllloooooo, i reached my task guys\
              ")
        return task_reached
    
    def raw_navigate(self, rel_sp, pos_tolerance, angle_tolerance, timeout=-1):
        rclpy.spin_once(self.node, timeout_sec = 1)
        nav_req = Pose()
        pose = copy.deepcopy(self.pose)
        print("pose = \n",self.pose)
        current_position = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        print(f"current pose = {current_position}")
        current_orien = Rotation.from_euler("ZYX", (pose.orientation.yaw, 0, 0), degrees=True)
        print(f"relative setpoint  = {rel_sp}")
        print(f"curr oreo  = {current_orien.as_euler('ZYX', degrees=True)}")
   
        rel_setpoint = np.array(rel_sp[:3])
        rel_orien = Rotation.from_euler("ZYX", (rel_sp[5], rel_sp[4], rel_sp[3]), degrees=True)
        print(f"rel oreo  = {rel_orien.as_euler('ZYX', degrees=True)}")

        global_setpoint = current_position + np.matmul(current_orien.as_matrix(), rel_setpoint)
        yaw, pitch, roll = (current_orien*rel_orien).as_euler("ZYX", degrees=True)
        print(f" global setpoint {global_setpoint}\n ayw = {yaw}")
        nav_req.position.x = float(global_setpoint[0])
        nav_req.position.y = float(global_setpoint[1])
        nav_req.position.z = float(global_setpoint[2])
        nav_req.orientation.roll = float(roll)
        nav_req.orientation.pitch = float(pitch)
        nav_req.orientation.yaw = float(yaw)
        pos_tolerance = float(pos_tolerance)
        angle_tolerance = float(angle_tolerance)
        self.in_transit = True
        self.current_service_type = 'align'

        self._action_client.wait_for_server()

        
        goal_msg = Interrupt.Goal()

        goal_msg.current_service_type = self.current_service_type
        goal_msg.setpoint = nav_req
        goal_msg.pos_tolerance = pos_tolerance
        goal_msg.angle_tolerance = angle_tolerance


        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        while(1):
            print(f"{self.result} is the result")
            time.sleep(0.1)
            rclpy.spin_once(self.node, timeout_sec = 1)
            if self.result:
                break


        # print(nav_req.pose)
        # for i in range(2):
        #     while not self.nav_align_proxy.wait_for_service(timeout_sec=2.0):
        #         print("Service unavailable")
            
        #     nav_res_align = self.nav_align_proxy.call_async(nav_req)
        #     print("$$$$$$$$$$$$$$$$$$4")
        #     rclpy.spin_until_future_complete(self.node, nav_res_align)
        #     self.current_nav_service_id += 1
        #     print("raw navigate align service", self.current_nav_service_id)
        #     time.sleep(10)
        # # self.current_nav_service_id+=1
        # # nav_req.service_id = 2

        # timeout = 10
        # start_time = time.time()
        # while True:
        #     cur_time = time.time()
        #     if timeout != -1 and cur_time - start_time > timeout:
        #         print("Timed out")
        #         break
        #     if not self.in_transit:
        #         break
        #     rclpy.spin_once(self.node)

        setpoint_reached = self.result
        self.result = False
        print(setpoint_reached, "helllloooooo, i navigated my task guys")
        return setpoint_reached

    async def align_to_task(self, alignment_params: list, timeout: float = -1):
        align_req = AlignService.Request()
        align_req.x = alignment_params[0]
        align_req.y = alignment_params[1]
        align_req.z = alignment_params[2]
        align_req.angle = alignment_params[3]

        self.in_transit = True
        align_res = await self.nav_align_proxy.call_async(align_req)
        self.current_nav_service_id = align_res.service_id

        start_time = time.time()
        while True:
            cur_time = time.time()
            if timeout != -1 and cur_time - start_time > timeout:
                break
            if not self.in_transit:
                break

        task_aligned = not self.in_transit
        return task_aligned

    async def scan_task(self, scan_params: list, timeout: float = -1):
        scan_req = ScanService.Request()
        scan_req.position.x = scan_params[0]
        scan_req.position.y = scan_params[1]
        scan_req.position.z = scan_params[2]
        scan_req.scan_radius = scan_params[3]

        self.in_transit = True
        scan_res = await self.nav_scan_proxy.call_async(scan_req)
        self.current_nav_service_id = scan_res.service_id

        start_time = time.time()
        while True:
            cur_time = time.time()
            if timeout != -1 and cur_time - start_time > timeout:
                break
            if not self.in_transit:
                break

        task_scanned = not self.in_transit
        return task_scanned

    async def fire_torpedo(self, torpedo_number: int):
        torpedo_req = TorpedoState.Request()
        torpedo_req.state = torpedo_number
        torpedo_res = await self.torpedo_proxy.call_async(torpedo_req)
        return torpedo_res

    async def drop_marker(self, marker_number: int):
        marker_req = MarkerDropperState.Request()
        marker_req.state = marker_number
        marker_res = await self.marker_dropper_proxy.call_async(marker_req)
        return marker_res

    async def move_gripper(self, gripper_state: int):
        gripper_req = GripperState.Request()
        gripper_req.state = gripper_state
        gripper_res = await self.gripper_proxy.call_async(gripper_req)
        return gripper_res

    async def acquire_pinger(self, frequency: float):
        pinger_req = PingerAcq.Request()
        pinger_req.frequency = frequency
        pinger_res = await self.acoustics_proxy.call_async(pinger_req)
        return pinger_res
    
    def scan_align_continuous(self, map, task_name, xy_tolerance, z_tolerance, angle_tolerance,
                         displacement, hard_z_value, scan_type="vertical_spiral",
                         timeout=3, align_wait_time=2):
        
        #$$$$$$$$
        # self.toggle_vision_task_wrapper(task_name)
        #$$$$$$$$$
        prev_setpoint = np.zeros(3, dtype=float)
        first_align = True
        count = 0
        goal_msg = Interrupt.Goal()
        goal_msg.current_service_type = 'scan'
        goal_msg.scan_type = scan_type
        goal_msg.xy_tolerance = float(xy_tolerance)
        goal_msg.z_tolerance = float(z_tolerance)
        goal_msg.angle_tolerance = float(angle_tolerance)
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        future = self._send_goal_future.add_done_callback(self.goal_response_callback)

        # nav_res = self.nav_scan_proxy(nav_req)
        # self.current_nav_service_id = nav_res.service_id
        
        print("scan service")
        # Scanning and waiting for the first detection
        start_time = time.time()
        while True:
            rclpy.spin_once(self.node, timeout_sec = 1)
            task_detected, task_location = self.check_vision(task_name)
            print(f"The task detected is {task_detected}")
            cur_time = time.time()
            print(f"time diff is {cur_time - start_time} , the timeout = {timeout}")
            if timeout != -1 and (cur_time - start_time) > timeout:
                print("Timed out")
                return False
            if task_detected:
                break

        
         # Aligning continuously
        print("Started aligning")
        req = PwmCap.Request()
        req.max_pwm = 1630
        # self.pwm_cap_proxy(req)

        start_time = time.time()
        while True:
            rclpy.spin_once(self.node, timeout_sec = 1)
            cur_time = time.time()
            if timeout != -1 and (cur_time - start_time) > timeout:
                print("Timed out")
                return False

            # Scanning for global position of task object
            task_detected, task_position = self.check_vision(task_name)
            if not task_detected:
                continue
            print("task_detected:" , task_detected)
            # Aligning
            # task_yaw = map.get_task_pose(task_name)[5]
            task_yaw = map[5]
            task_position = np.array(task_position)
            if np.linalg.norm(
                task_position-
                np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
                ) >2000:
                print("False Detection")
                continue
            task_orien = Rotation.from_euler("ZYX", (task_yaw, 0.0, 0.0), degrees=True)
            
            disp_position = np.array(displacement[:3]) #from yaml
            disp_orien = Rotation.from_euler("ZYX", (displacement[5], displacement[4], displacement[3]), degrees=True)

            global_setpoint = task_position + np.matmul(task_orien.as_matrix(), disp_position)
            if hard_z_value is not None:
                print("Hardcoded z to:", hard_z_value)
                global_setpoint[2] = hard_z_value

            print(global_setpoint)
            print("task_position:", task_position)
            print("global_setpoint:", global_setpoint)
            yaw, pitch, roll = (task_orien*disp_orien).as_euler("ZYX", degrees=True)
            
            ### action starting
            nav_req = Interrupt.Goal()
            nav_req.current_service_type = 'align'
            nav_req.setpoint.position.x = float(global_setpoint[0])
            nav_req.setpoint.position.y = float(global_setpoint[1])
            nav_req.setpoint.position.z = float(global_setpoint[2])
            nav_req.setpoint.orientation.roll = float(roll)
            nav_req.setpoint.orientation.pitch = float(pitch)
            nav_req.setpoint.orientation.yaw = float(yaw)
            nav_req.pos_tolerance = float(xy_tolerance) #from yaml
            nav_req.angle_tolerance = float(angle_tolerance) #from yaml
            
            ### add action here 
            # self.in_transit = True
            # nav_res = self.nav_align_proxy(nav_req)
            # self.current_nav_service_id = nav_res.service_id
            # print("align service", self.current_nav_service_id)
            self._send_goal_future = self._action_client.send_goal_async(
            nav_req,
            feedback_callback=self.feedback_callback)
            future = self._send_goal_future.add_done_callback(self.goal_response_callback)

            # just testing remaining 
            print("aligning")
            time.sleep(0.5)
            print(global_setpoint, roll, pitch, yaw)
            
            curr_pos = copy.deepcopy(self.pose)
            xy_dist = np.linalg.norm(
                np.array([curr_pos.position.x, curr_pos.position.y]) -
                np.array([global_setpoint[0], global_setpoint[1]])
            )
            z_dist = np.abs(curr_pos.position.z - global_setpoint[2])
            print("count:", count)
            print("xy_dist:", xy_dist)
            print("z_dist:", z_dist)
            print("xy_tolerance:", xy_tolerance)
            print("z_tolerance:", z_tolerance)
            if xy_dist <= xy_tolerance and z_dist <= z_tolerance:
                count += 1
            else:
                count = max(0, count-1)
            if count >= 7:
                break

        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        req = PwmCap.Request()
        req.max_pwm = 1780
        # self.pwm_cap_proxy(req)

        # Updating the map after verifying
        # map.update_task(task_name, prev_setpoint)
        return True

    #self, task_name, task_location, pos_tolerance, angle_tolerance, timeout=-1