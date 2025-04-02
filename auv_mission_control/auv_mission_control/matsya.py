import copy
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import asyncio
import time
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
from auv_msgs.srv import SetDetector
from auv_msgs.msg import CurrentNavId
from rclpy.executors import SingleThreadedExecutor


class Matsya:#(Node):
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()

        self.logger.info("Initializing Matsya...")

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
        except Exception as e:
            self.logger.error(f"Error initializing service clients: {e}")
            raise

        # Initialize subscribers
        self.nav_status_sub = self.node.create_subscription(NavStatus, "/navigator/status", self.cb_nav, 10)
        self.vision_sub = self.node.create_subscription(VisionSetpoints, "/vision/detection", self.cb_vision, 10)
        self.localization_sub = self.node.create_subscription(AuvState, "/localization/pose", self.cb_pose, 10)
        self.nav_status = self.node.create_publisher(CurrentNavId,"/mission_control/current_service_id", 1)

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

    def cb_pose(self, msg: AuvState):
        self.pose = msg
        self.pose_is_valid = True
        print("RECEIVED AND WORKING AS EXPECTED")
        #self.logger.info(f"Updated Matsya pose: {msg}")
        

    def cb_vision(self, msg: VisionSetpoints):
        self.vision_detections = msg

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

    async def check_vision(self, task_name: str):
        req = Empty.Request()
        await self.basler_on_proxy.call_async(req)
        self.node.get_logger().info('Camera On')
        await asyncio.sleep(1)
        detections = copy.deepcopy(self.vision_detections.tasks)
        for i in range(len(detections)):
            if detections[i].name != task_name:
                continue

            task_detected = True
            task_location = [detections[i].position.x,
                             detections[i].position.y,
                             detections[i].position.z]
            await self.basler_off_proxy.call_async(req)
            return task_detected, task_location
        await self.basler_off_proxy.call_async(req)
        return False, [0.0, 0.0, 0.0]

    def reach_task(self, task_location: list, pos_tolerance: float, angle_tolerance: float, timeout: float = -1):
        nav_req = SetpointService.Request()
        nav_req.pose.position.x = float(task_location['x'])
        nav_req.pose.position.y = float(task_location['y'])
        nav_req.pose.position.z = float(task_location['z'])
        nav_req.pose.orientation.roll = float(task_location['roll'])
        nav_req.pose.orientation.pitch = float(task_location['pitch'])
        nav_req.pose.orientation.yaw = float(task_location['yaw'])
        nav_req.pos_tolerance = float(pos_tolerance)
        nav_req.angle_tolerance = float(angle_tolerance)
        self.in_transit = True

        while not self.nav_setpoint_proxy.wait_for_service(timeout_sec=1.0):
            print("Service unavailable")
        nav_fut = self.nav_setpoint_proxy.call_async(nav_req)
        print("waiting")
        # return nav_fut
        rclpy.spin_until_future_complete(self.node, nav_fut)
        # self.current_nav_service_id+=1
        print("setpoint service", self.current_nav_service_id)
        self.current_nav_service_id += 1
        # if not self.in_transit:
        #     break

        timeout = 5
        start_time = time.time()
        while True:
            # print("i am in while true")
            cur_time = time.time()
            if timeout != -1 and cur_time - start_time > timeout:
                break
            if not self.in_transit:
                break
            for i in range(1):
                if rclpy.ok():
                    rclpy.spin_once(self.node, timeout_sec = 1)  # Wait briefly for new messages
                    time.sleep(0.1)  # Avoid high CPU usage

        task_reached = not self.in_transit
        print(task_reached, "helllloooooo, i reached my task guys\
              ")
        return task_reached
    
    def raw_navigate(self, rel_sp, pos_tolerance, angle_tolerance, timeout=-1):
        nav_req = AlignService.Request()
        pose = copy.deepcopy(self.pose)
        current_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        current_orien = Rotation.from_euler("ZYX", (pose.orientation.yaw, 0, 0), degrees=True)
        
        rel_setpoint = np.array(rel_sp[:3])
        rel_orien = Rotation.from_euler("ZYX", (rel_sp[5], rel_sp[4], rel_sp[3]), degrees=True)

        global_setpoint = current_position + np.matmul(current_orien.as_matrix(), rel_setpoint)
        yaw, pitch, roll = (current_orien*rel_orien).as_euler("ZYX", degrees=True)
        nav_req.pose.position.x = float(global_setpoint[0])
        nav_req.pose.position.y = float(global_setpoint[1])
        nav_req.pose.position.z = float(global_setpoint[2])
        nav_req.pose.orientation.roll = float(roll)
        nav_req.pose.orientation.pitch = float(pitch)
        nav_req.pose.orientation.yaw = float(yaw)
        nav_req.pos_tolerance = float(pos_tolerance)
        nav_req.angle_tolerance = float(angle_tolerance)
        self.in_transit = True
        print(nav_req.pose)
        for i in range(2):
            while not self.nav_align_proxy.wait_for_service(timeout_sec=2.0):
                print("Service unavailable")
            
            nav_res_align = self.nav_align_proxy.call_async(nav_req)
            print("$$$$$$$$$$$$$$$$$$4")
            rclpy.spin_until_future_complete(self.node, nav_res_align)
            self.current_nav_service_id += 1
            print("raw navigate align service", self.current_nav_service_id)
            time.sleep(10)
        # self.current_nav_service_id+=1
        # nav_req.service_id = 2

        timeout = 10
        start_time = time.time()
        while True:
            cur_time = time.time()
            if timeout != -1 and cur_time - start_time > timeout:
                print("Timed out")
                break
            if not self.in_transit:
                break
            rclpy.spin_once(self.node)

        print("##################3")
        setpoint_reached = not self.in_transit
        print("##################44444444444444444444") 
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