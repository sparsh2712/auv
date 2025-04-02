import rclpy
import copy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation
import pickle
from auv_msgs.msg import DVLOrient
from auv_msgs.msg import DVLVel
from auv_msgs.msg import PsData
from auv_msgs.msg import AuvState
from auv_msgs.msg import Pose
from auv_msgs.msg import ThrusterForces
from auv_msgs.msg import GlobalForces
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import zmq
from std_srvs.srv import Empty
import os
from auv_msgs.srv import TorpedoState, GripperState, MarkerDropperState
from auv_msgs.msg import PwmData
import numpy as np
import time
import traceback

    


class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')

        self.ALL = b'ALL'
        self.USB = b'USB'
        self.ROS = b'ROS'
        self.GUI = b'GUI'

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.DEALER)
        self.socket.identity = self.ROS  # Unique identity for USB Driver
        self.socket.connect("tcp://localhost:5555")  # Connect to Router

        self.last_read_time = time.time_ns()
        self.poll_time_ns = 1_000_000  # 1 ms

        self.router_read_function_mappings = {
            "imu_data": self.publish_imu,
            "PS_depth": self.publish_ps,
            "localisation_reset": self.localization_reset,
            "port_list": self.empty_function,
            "mission_isr_pressed": self.empty_function,
            "marker_state": self.recieved_marker_state,
            "torpedo_state": self.recieved_torpedo_state,
            "gripper_state": self.recieved_gripper_state,
            "serial_port_changed": self.empty_function,
            "software_driver_log": self.empty_function,
            "elec_stack_log": self.empty_function,
            "file_system": self.cb_file_system_functions,
            "software_driver_log": self.print_log,
            "elec_stack_log": self.print_log

        }

        self.software_subdirectory_path = "/home/matsya/ros2_ws/src/Robosub_ROS2_2024/"

        # "software_driver_log" -> string
        # "software_driver_log_hex" -> string
        # "port_list" -> list of ports as strings
        # "serial_port_changed" -> string (new port name)
        # "PS_depth" -> float
        # "mission_isr_pressed" -> integer: 1
        # "marker_state" -> integer
        # "torpedo_state" -> integer
        # "gripper_state" -> integer
        # "imu_data" -> list of 4 floats (quaternion x, y, z, w)

        self.max_pwm = 1900 # have to make it such that it imports from yaml of controller
        self.min_pwm = 1100
        self.pose = AuvState()

        # Subscriptions
        self.dvl_vel_sub = self.create_subscription(DVLVel, '/dvl/velData', self.cb_dvl_vel,10)
        self.dvl_orient_sub = self.create_subscription(DVLOrient,'/dvl/orientData',self.cb_dvl_orient,10)
        self.belief_pub = self.create_subscription(AuvState, '/localization/pose', self.cb_pose, 10)
        self.setpoint_sub = self.create_subscription(Pose, '/controller/setpoint', self.cb_setpoint,10)
        self.thruster_pwm_sub = self.create_subscription(PwmData, '/controller/pwm', self.cb_Pwm,10)
        self.global_forces_sub = self.create_subsciption(GlobalForces, '/controller/global_forces', self.cb_global_forces, 10)
        self.thrust_force_sub = self.create_publisher(ThrusterForces, '/controller/thruster_forces', self.cb_thrust_forces, 10)

        # Publishers
        self.setpoint_pub = self.create_publisher(Pose, '/controller/setpoint', 10)
        self.publish_imu_data = self.create_publisher(Imu, '/imu/data', 10)
        self.Ps_pub = self.create_publisher(PsData, '/ps/data', 10)
        
        # things we need: 
        # all sensor data and pose: done
        # pwm: done
        # publish and subscribe setpoint: done
        # 

        # Services
        self.torpedo_state=self.create_service(TorpedoState, "/actuators/torpedo", self.cb_torpedo)
        self.gripper_state=self.create_service(GripperState, "/actuators/gripper", self.cb_gripper)
        self.marker_dropper_state=self.create_service(MarkerDropperState,"/actuators/marker_dropper",  self.cb_marker_dropper)
        self.unkill_thrusters=self.create_service(Empty, "/thruster/unkill", self.cb_unkill_thrusters)
        self.kill_thrusters=self.create_service(Empty, "/thruster/kill", self.cb_kill_thrusters)
        
        # self.shoot_torpedo = self.create_service(Empty, '/actuators/torpedo', self.cb_shoot_torpedo)
        # self.drop_marker = self.create_service(Empty, '/actuators/marker_dropper', self.cb_drop_marker)
        # self.grip = self.create_service(Empty, '/actuators/gripper', self.cb_grip)
        
        # Service clients
        self.client = self.create_client(Empty, '/localization/reset_service')

    def send_to_router(self, receiver, data_type, data):
        # For receiver, pls use only self.ROS, self.GUI and self.ALL
        # Data type is a single python string
        # Data can be anything
        data = pickle.dumps(data)
        message = [receiver, data_type.encode(), data]
        self.socket.send_multipart(message)


    def read_from_router(self):
        if (time.time_ns() - self.last_read_time) < self.poll_time_ns:
            return  

        try:
            msg = self.socket.recv_multipart(zmq.NOBLOCK)
            if len(msg) != 3:
                self.get_logger().info("Invalid message received from router")
                return

            msg_receive_time = time.time_ns()
            sender = msg[0]
            dtype = msg[1].decode()
            data = pickle.loads(msg[2])
            self.execute_function(dtype, data)

            latency = (time.time_ns() - msg_receive_time) / 1_000_000
            self.get_logger().info(f"ZMQ Msg Latency: {latency:.2f}ms")

        except zmq.Again:
            pass  # No message available
        except Exception as e:
            self.get_logger().error(f"ZMQ Error: {traceback.format_exc()}")

        self.last_read_time = time.time_ns()  # Update last ZMQ poll time

    def execute_function(self, func_name, *args, **kwargs):
        if func_name in self.router_read_function_mappings:
            return self.router_read_function_mappings[func_name](*args, **kwargs)
        else:
            self.get_logger().info(f"Function '{func_name}' not found.")

    def send_actuator_message():
        pass

    def recieved_torpedo_state(self, data):
        torpedo_state = data

    def recieved_gripper_state(self, data):
        gripper_state = data

    def recieved_marker_state(self, data):
        marker_state = data

    # def cb_shoot_torpedo(self, req, response):
    #     self.send_to_router(self.USB, "command", ["shoot_torpedo"])
    #     return response
    
    # def cb_drop_marker(self, req, response):
    #     self.send_to_router(self.USB, "command", ["drop_marker"])
    #     return response
    
    # def cb_grip(self, req, response):
    #     self.send_to_router(self.USB, "command", ["grip"])
    #     return response


    def send_request(self):
        request = Empty.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)
        
    def callback(self, future):
        try:
            future.result()
            self.get_logger().info('Service call succeeded')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def localization_reset(self):
        #some condition so that sevice is called when recieved a signal to do so from router
        self.send_request()

    def publish_imu(self, data):
        # data = self.recieve_from_router()
        msg = Imu()

        # Assign quaternion values
        msg.orientation.x = data[0]
        msg.orientation.y = data[1]
        msg.orientation.z = data[2]
        msg.orientation.w = data[3]
        self.publish_imu_data.publish(msg)

    def publish_ps(self, data):
        # data = self.recieve_from_router()
        msg = PsData()
        msg.depth = float(data)
        self.Ps_pub.publish(msg)
        

    def cb_torpedo(self, req, response):
        self.send_to_router(self.USB,"command",['get_torpedo_state'])
        return response
    
    def cb_gripper(self, req, response):
        self.send_to_router(self.USB,"command",['get_gripper_state'])
        return response
    
    def cb_marker_dropper(self, req, response):
        self.send_to_router(self.USB,"command",['get_marker_state'])
        return response
    
    def cb_unkill_thrusters(self, req, response):
        self.get_logger().info('unkilling')
        self.send_to_router(self.USB, "command", ['unkill'])
        return response
    
    def cb_kill_thrusters(self, req, response):
        self.send_to_router(self.USB, "command", ['soft_kill'])
        return response


    def cb_pose(self, msg):
        state = {
            'position': {
                'x': msg.position.x,
                'y': msg.position.y,
                'z': msg.position.z
            },
            'velocity': {
                'x': msg.velocity.x,
                'y': msg.velocity.y,
                'z': msg.velocity.z
            },
            'acceleration': {
                'x': msg.acceleration.x,
                'y': msg.acceleration.y,
                'z': msg.acceleration.z
            },
            'orientation': {
                'roll': msg.orientation.roll,
                'pitch': msg.orientation.pitch,
                'yaw': msg.orientation.yaw
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'height': msg.height,
            'depth': msg.depth
        }
        self.send_to_router(self.GUI, "auv_pose", state)


    def cb_Pwm(self, msg):
        pwm_msg=[]
        pwm_msg.append(msg.surge_back_left)
        pwm_msg.append(3000 - msg.heave_front_left)
        pwm_msg.append(msg.heave_back_left)
        pwm_msg.append(3000 - msg.surge_front_left)
        pwm_msg.append(3000 - msg.surge_front_right)
        pwm_msg.append(msg.heave_front_right)
        pwm_msg.append(msg.heave_back_right)
        pwm_msg.append(msg.surge_back_right)

        self.send_to_router(self.USB, "command", ['send_thruster_pwms', [pwm_msg]])


        # self.dvl_vel_sub = self.create_subscription(DVLVel, '/dvl/velData', self.cb_dvl_vel,10)
        # self.dvl_orient_sub = self.create_subscription(DVLOrient,'/dvl/orientData',self.cb_dvl_orient,10)
        # self.belief_pub = self.create_subscription(AuvState, '/localization/pose', self.cb_pose, 10)
        # self.setpoint_sub = self.create_subscription(Pose, '/controller/setpoint', self.cb_setpoint,10)
        # self.thruster_pwm_sub = self.create_subscription(PwmData, '/controller/pwm', self.cb_Pwm,10)
        # self.global_forces_sub = self.create_subsciption(GlobalForces, '/controller/global_forces', self.cb_global_forces, 10)
        # self.thrust_force_sub = self.create_publisher(ThrusterForces, '/controller/thruster_forces', self.cb_thrust_forces, 10)

    def cb_dvl_vel(self, msg):
        # DVLVel
        # self.send_to_router(self.GUI, "dvl_velocity", state)
        pass

    def cb_dvl_orient(self, msg):
        # DVLOrient
        # self.send_to_router(self.GUI, "dvl_orientation", state)
        pass

    def cb_setpoint(self, msg):
        # Pose
        # self.send_to_router(self.GUI, "global_setpoint", state)
        pass

    def cb_global_forces(self, msg):
        # GlobalForces
        # self.send_to_router(self.GUI, "global_forces", state)
        pass

    def cb_thrust_forces(self, msg):
        # ThrusterForces
        # self.send_to_router(self.GUI, "thruster_forces", state)
        pass


    def print_log(self, data):
        print(data)
    
    #Have to be directly called in main for these functions to be executed:
    def verbose_on(self):
        self.send_to_router(self.USB, "command", ["verbose_on"])

    def verbose_off(self):
        self.send_to_router(self.USB, "command", ["verbose_off"])
        
    def send_current_time(self):
        self.send_to_router(self.USB, "command", ["send_current_time"])

    def get_comport(self):
        self.send_to_router(self.USB, "command", ["get_comport"])

    def change_port(self, portname):
        self.send_to_router(self.USB, "command", ['change_port', portname])

    def set_thruster_pwm_timeout(self, timeout):
        self.send_to_router(self.USB,"command",["set_thruster_pwm_timeout",timeout])

    def set_max_pwm(self):
        self.send_to_router(self.USB,"command",["set_thruster_pwm_max", self.max_pwm]) 
    
    def set_min_pwm(self):
        self.send_to_router(self.USB, "command", ["set_thruster_pwm_min", self.min_pwm])

    def get_current_port(self):
        self.send_to_router(self.USB, "command", ["get_current_port"])

    def debug_on(self):
        self.send_to_router(self.USB, "command", ["debug_on"])

    def debug_off(self):
        self.send_to_router(self.USB, "command", ["debug_off"])

    def power_off(self, argument):
        '''argument can be:
            "SBC_POWER"
            "DVL_POWER"
            "SPARE_5V"
            "THRUSTER_RELAY"
            "SPARE_16V_1"
            "SPARE_16V_2"
            "LED_STRIP"
            "SCREEN"
            "SERVO_POWER"
        '''
        self.send_to_router(self.USB, "command", ["power_off", [argument]])#since argument itself is a string 

    def power_on(self, argument):
        '''argument can be:
            "SBC_POWER"
            "DVL_POWER"
            "SPARE_5V"
            "THRUSTER_RELAY"
            "SPARE_16V_1"
            "SPARE_16V_2"
            "LED_STRIP"
            "SCREEN"
            "SERVO_POWER"
        '''
        self.send_to_router(self.USB, "command", ["power_on", [argument]])#since argument itself is a string 


    #in debug mode
    def send_thruster_individual_pwm(self, thruster_index, pwm):
        self.send_to_router(self.USB, "command", ["send_thruster_individual_pwm", [thruster_index,pwm]])
    
    def send_servo_individual_pwm(self, servo_type, pwm):
        self.send_to_router(self.USB, "command", ["send_servo_individual_pwm", [servo_type, pwm]])

    def empty_function(self, *args, **kwargs):
        pass

    
    

    # def get_marker_state():
    #     self.send_to_router(self.USB,"command",['get_marker_state'])
    # def get_gripper_state():
    #     self.send_to_router(self.USB,"command",['get_gripper_state'])
    # def get_torpedo_state():
    #     self.send_to_router(self.USB,"command",['get_torpedo_state'])

    # def test(self):
    #     self.send_to_router(self.USB, "command", ['get_torpedo_state'])


    # def cb_torpedo(self, req):
    #     self.send_to_router()

    # def cb_gripper(self, req):
    #     self.send_to_router()

    # def cb_marker_dropper(self, req):
    #     self.send_to_router()

    # def reset_marker_state():
    #     self.send_to_router()

    def cb_file_system_functions(self, msg):
        file_command = msg[0]
        data = msg[1]

        if file_command == "read_file":
            path = data
            try:
                with open(path, "rb") as f:
                    file_data = f.read()
            except:
                print("Error opening file")
            
            self.send_to_router(self.GUI, "file_system", ["file_data", [path, file_data]])

        elif file_command == "get_subdirectory_tree":
            tree = self.get_directory_tree(self.software_subdirectory_path)
            data = ["subdirectory_tree", tree]
            self.send_to_router(self.GUI, "file_system", data)
        
        elif file_command == "save_file":
            path = data[0]
            file_contents = data[1]
            print("saving")
            try:
                with open(path, "w") as file:
                    file.write(file_contents)
                    print(f"File saved at {path}")
            except:
                print("File couldn't be saved")


    def get_directory_tree(self, root_path):
        if not os.path.exists(root_path):
            raise ValueError(f"Path {root_path} does not exist.")
        
        def build_tree(path):
            tree = {
                "type": "subdirectory" if os.path.isdir(path) else "file",
                "path": os.path.abspath(path)
            }
            if os.path.isdir(path):
                tree["children"] = []
                try:
                    for entry in sorted(os.listdir(path)):
                        full_path = os.path.join(path, entry)
                        tree["children"].append(build_tree(full_path))
                except Exception as e:
                    tree["error"] = str(e)
            return tree
        
        return build_tree(root_path)



def main(args=None):
    print("LESSS GO")
    rclpy.init(args=args)
    driver_node = DriverNode()
    print('before')
    driver_node.send_to_router(driver_node.USB, "command", ["change_port", ["/dev/ttyACM0"]])
    print('after')
        
    while rclpy.ok():
        try:
            rclpy.spin_once(driver_node, timeout_sec=0.000001)  # Run ROS callbacks first
            driver_node.read_from_router()  

            # change above to zmq polling instead of manually polling, but this works fine too

        except KeyboardInterrupt:
            driver_node.get_logger().info("Shutting down node")
            break

    driver_node.destroy_node()
    rclpy.shutdown()

    
        

if __name__ == '__main__':
    main()





