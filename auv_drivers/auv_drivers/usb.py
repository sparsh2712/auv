import time
import serial
import serial.tools.list_ports
import struct
import zmq
import pickle
import traceback
import sys

# Message IDs
TUSB_MSG_ID_THRUSTER_SEND_PWMS =                0x01
TUSB_MSG_ID_THRUSTER_SEND_INDIVIDUAL_PWM =      0x02
TUSB_MSG_ID_THRUSTER_KILL_UNKILL =              0x03
TUSB_MSG_ID_THRUSTER_CHANGE_LIMITS =            0x04
TUSB_MSG_ID_THRUSTER_CHANGE_PWM_TIMEOUT =       0x05
TUSB_MSG_ID_SERVO_ACTUATE =                     0x06
TUSB_MSG_ID_SERVO_RESET =                       0x07
TUSB_MSG_ID_SERVO_SEND_CUSTOM_PWM =             0x08
TUSB_MSG_ID_DATA_IMU =                          0x09
TUSB_MSG_ID_DATA_PS =                           0x0A
TUSB_MSG_ID_AUV_STATE =                         0x0B
TUSB_MSG_ID_LOG_STATEMENTS =                    0x0C
TUSB_MSG_ID_GET_COMMANDS =                      0x0D
TUSB_MSG_ID_BATTERY_WARNING =                   0x0E
TUSB_MSG_ID_TEMPERATURE_WARNING =               0x10
TUSB_MSG_ID_TOGGLE =                            0x11
TUSB_MSG_ID_UNIX_TIME =                         0x12
TUSB_MSG_ID_MISSION_START =                     0x13
TUSB_MSG_ID_RESET_IMU =                         0x14
TUSB_MSG_ID_CURRENT_TIME_REQUEST =              0x15

# Reserved Bytes
TUSB_MSG_START =                                0xF2
TUSB_MSG_STOP =                                 0xEB

# Toggle IDs for Verbose, Debug, and Mosfets
TOGGLE_ID_VERBOSE =                             0x2A
TOGGLE_ID_DEBUG =                               0x2B

TOGGLE_ID_SBC_POWER =                           0x20
TOGGLE_ID_DVL_POWER =                           0x21
TOGGLE_ID_SPARE_5V =                            0x22
TOGGLE_ID_THRUSTER_RELAY =                      0x23
TOGGLE_ID_SPARE_16V_1 =                         0x24
TOGGLE_ID_SPARE_16V_2 =                         0x25
TOGGLE_ID_LED_STRIP =                           0x26
TOGGLE_ID_SCREEN =                              0x27
TOGGLE_ID_SERVO_POWER =                         0x28

GET_COMMAND_ID_MARKER_STATE =                   0x01
GET_COMMAND_ID_TORPEDO_STATE =                  0x02
GET_COMMAND_ID_GRIPPER_STATE =                  0x03
GET_COMMAND_ID_BATTERY_VOLTAGE_THRUSTERS =      0x04
GET_COMMAND_ID_BATTERY_VOLTAGE_STACK =          0x05
GET_COMMAND_ID_SECOND_PICO_CONNECTED =          0x06

SOFT_KILL =                                     0x01
UNKILL =                                        0x02
CHANGE_THRUSTER_MAX =                           0x01
CHANGE_THRUSTER_MIN =                           0x02
MARKER_DROPPER  =                               0x01
TORPEDO_SHOOTER =                               0x02
GRIPPER         =                               0x03


class USBDriver():
    

    def __init__(self, port=None, baudrate=921600, timeout=0.1):
        super().__init__()

        self.pause_read = False
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.identity = b'USB' 
        self.socket.connect("tcp://localhost:5555")  

        self.ALL = b'ALL'
        self.USB = b'USB'
        self.ROS = b'ROS'
        self.GUI = b'GUI'
        self.serial_read_time_period = 1_000_000 # nanoseconds (read_message called once every these many nanoseconds)
        self.last_read_time = time.time_ns()

        print("USB Driver: Connected to Router")

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        if self.port is not None:
            self.ser = serial.Serial(port=self.port, baudrate=baudrate, timeout=timeout)
            self.software_driver_log("USB available")
            print("USB available")
        else:
            self.ser = None
            self.software_driver_log("USB not available")
            print("USB not available")

        self.counter=0 

        self.function_mappings = {
            "get_com_ports": self.get_com_ports,
            "change_port": self.change_port,
            "send_thruster_pwms": self.send_thruster_pwms,
            "send_thruster_individual_pwm": self.send_thruster_individual_pwm,
            "soft_kill": self.soft_kill,
            "unkill": self.unkill,
            "set_thruster_pwm_max": self.set_thruster_pwm_max,
            "set_thruster_pwm_min": self.set_thruster_pwm_min,
            "set_thruster_pwm_timeout": self.set_thuster_pwm_timeout,
            "drop_marker": self.drop_marker,
            "shoot_torpedo": self.shoot_torpedo,
            "grip": self.grip,
            "get_marker_state": self.get_marker_state,
            "get_torpedo_state": self.get_torpedo_state,
            "get_gripper_state": self.get_gripper_state,
            "send_servo_individual_pwm": self.send_servo_individual_pwm,
            "reset_marker_dropper": self.reset_marker_dropper,
            "reset_torpedo_shooter": self.reset_torpedo_shooter,
            "reset_gripper": self.reset_gripper,
            "send_auv_state": self.send_auv_state,
            "send_current_time": self.send_current_time,
            "debug_on": self.debug_on,
            "debug_off": self.debug_off,
            "verbose_on": self.verbose_on,
            "verbose_off": self.verbose_off,
            "power_on": self.power_on,
            "power_off": self.power_off,
            "get_current_port": self.send_current_port,
            "reset_imu": self.reset_imu
        }

        self.toggle_mappings = {
            "SBC_POWER": TOGGLE_ID_SBC_POWER,
            "DVL_POWER": TOGGLE_ID_DVL_POWER,
            "SPARE_5V": TOGGLE_ID_SPARE_5V,
            "THRUSTER_RELAY": TOGGLE_ID_THRUSTER_RELAY,
            "SPARE_16V_1": TOGGLE_ID_SPARE_16V_1,
            "SPARE_16V_2": TOGGLE_ID_SPARE_16V_2,
            "LED_STRIP": TOGGLE_ID_LED_STRIP,
            "SCREEN": TOGGLE_ID_SCREEN,
            "SERVO_POWER": TOGGLE_ID_SERVO_POWER
        }

        # data types: (related to USB driver only)
        # "command" -> list, with 0th index as function name, and optionally arguments in 1st index as LIST ONLY even if 1 argument
        # "software_driver_log" -> string
        # "software_driver_log_hex" -> string
        # "port_list" -> list of ports as strings
        # "serial_port_changed" -> string (new port name)
        # "PS_depth" -> float
        # "mission_isr_pressed" -> integer: 1
        # "marker_state" -> integer
        # "torpedo_state" -> integer
        # "gripper_state" -> integer
        # "imu_data" -> list of floats x, y, z, w (quaternion)

    def run(self):
        while True:
            try:
                # change below to a polling implementation but this works too

                msg = self.socket.recv_multipart(zmq.NOBLOCK)
                # sender, "command", [function_name, [args]]
                if len(msg) != 3: 
                    continue

                self.msg_receive_time = time.time_ns()
                sender = msg[0]
                dtype = msg[1].decode()
                data = pickle.loads(msg[2])
                if isinstance(data, list):
                    function_name = data[0]
                    if len(data) > 1:
                        args_list = data[1]
                        self.execute_function(function_name, *args_list)
                    else:
                        self.execute_function(function_name)
                else:
                    self.software_driver_log("Not a valid message to usb driver.")

                # print(f"Msg received to msg send: {(time.time_ns() - self.msg_receive_time) / 1000000}ms")
            except zmq.Again:
                pass
            except Exception as e:
                error = traceback.format_exc()
                self.software_driver_log(error)
                print(error)

            now = time.time_ns()
            if (now - self.last_read_time) >= self.serial_read_time_period:  # 1 millisecond = 1,000,000 ns
                self.read_message()
                self.last_read_time = now

            # if something doesn't work then add a time.sleep


    def send_to_router(self, receiver, data_type, data):
        # For receiver, pls use only self.ROS, self.GUI and self.ALL
        # Data type is a single python string
        # Data can be anything

        data = pickle.dumps(data)
        message = [receiver, data_type.encode(), data]
        self.socket.send_multipart(message)

    def execute_function(self, func_name, *args, **kwargs):
        if func_name in self.function_mappings:
            return self.function_mappings[func_name](*args, **kwargs)
        else:
            print(f"Function '{func_name}' not found.")
            self.software_driver_log(f"Function '{func_name}' not found.")

    def get_com_ports(self):
        ports = serial.tools.list_ports.comports()
        ports_string_list = [port.device for port in ports]
        self.send_to_router(self.ALL, "port_list", ports_string_list)
        return ports_string_list
    
    def send_current_port(self):
        self.send_serial_port_change_signal(self.port)

    def change_port(self, port):
        if port == "None":
            self.software_driver_log("Disconnecting serial port")
            print("Disconnecting serial port")
            self.pause_read = True
            if self.ser is not None:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.ser.close()
            self.ser = None
            self.port = None
            self.software_driver_log(f"Serial port disconnected")
            print(f"Serial port disconnected")
            self.send_serial_port_change_signal("None")
            time.sleep(0.1)
            self.pause_read = False
            return

        # self.software_driver_log(port)
        ports = self.get_com_ports()
        if port in ports:
            self.software_driver_log("Changing serial port")
            print("Changing serial port")
            self.pause_read = True
            if self.ser is not None:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.ser.close()
            try:
                self.ser = serial.Serial(port = port, baudrate = self.baudrate, timeout = self.timeout)
            except Exception as e:
                print(e)
                self.software_driver_log("Error: Could not connect to serial port")
                print("Error: Could not connect to serial port")
                self.pause_read = False
                return
            
            self.port = port
            self.software_driver_log(f"Serial port set to {port}")
            print(f"Serial port set to {port}")
            self.send_serial_port_change_signal(port)
            time.sleep(0.1)
            self.pause_read = False
        else:
            self.software_driver_log(f"No such port exists: {port}")
            print(f"No such port exists: {port}")

    def software_driver_log(self, message):
        self.send_to_router(self.ALL, "software_driver_log", message)

    def software_driver_log_hex(self, message):
        self.send_to_router(self.ALL, "software_driver_log_hex", message)

    def elec_stack_log(self, message):
        self.send_to_router(self.ALL, "elec_stack_log", message)

    def ps_receive_cb(self, depth):
        self.send_to_router(self.ALL, "PS_depth", depth)
        # print(f"Depth: {depth}")

    def imu_receive_cb(self, x, y, z, w):
        self.send_to_router(self.ALL, "imu_data", [x, y, z, w])

    def send_serial_port_change_signal(self, message):
        self.send_to_router(self.ALL, "serial_port_changed", message)
    
    def mission_isr_pressed_cb(self):
        self.send_to_router(self.ALL, "mission_isr_pressed", 1)
        self.elec_stack_log("Mission ISR pressed")
        print("Mission ISR pressed")

    def display_send_message(self, packet):
        hex_string = " ".join("%02x" % b for b in packet)
        self.software_driver_log_hex(hex_string)
        print(hex_string)

    def send_message(self, message_id, payload):
        for data in payload:
            if data > 255 or data < 0:
                self.software_driver_log("Wrong type of message")
                print("Wrong type of message")
                break
        payload = bytearray(payload)
        message_length = len(payload)
        checksum = (message_id ^ message_length)
        for byte in payload:
            checksum ^= byte

        packet = bytearray([TUSB_MSG_START, message_id, message_length])
        packet.extend(payload) 
        packet.extend(bytearray([checksum, TUSB_MSG_STOP]))
        if self.ser is not None:
            if not self.ser.closed:
                self.ser.write(packet)
        else:
            self.software_driver_log("USB not connected (no message sent)")
            print("USB not connected (no message sent)")
        self.display_send_message(packet)

    def read_message(self):
        try:
            if self.ser is None:
                return None
            elif self.pause_read:
                return None
            elif self.ser.closed:
                return None
            elif self.ser.in_waiting == 0:
                return None
            
            packet = self.ser.read_until(b'\xeb')
            start_index = 0
            for i in range(len(packet)):
                if packet[i] == TUSB_MSG_START:
                    break
                start_index += 1
            if start_index == len(packet):
                self.software_driver_log("Unable to find start byte")
                print("Unable to find start byte")
                return None
            
            try:
                message_id = packet[start_index + 1]
                payload_length = packet[start_index + 2] 
                payload = packet[start_index + 3 : -2]
                checksum = packet[-2]
            except IndexError:
                self.software_driver_log("IndexError: ")
                print("IndexError: ")
                self.display_send_message(packet)
                return None

            if not self.validate_message(message_id, payload_length, payload, checksum):
                self.software_driver_log("Invalid Message Received (Checksum failed)")
                print("Invalid Message Received (Checksum failed)")
                self.display_send_message(packet)
                return None
            
            # TUSB_MSG_ID_BATTERY_WARNING =                   0x0E
            # TUSB_MSG_ID_TEMPERATURE_WARNING =               0x10

            if message_id == TUSB_MSG_ID_DATA_PS:
                depth = self.unpack_float(payload)
                self.ps_receive_cb(depth)

            elif message_id == TUSB_MSG_ID_LOG_STATEMENTS:
                self.elec_stack_log(payload.decode("utf-8")[:-1])
                print(payload.decode("utf-8")[:-1])

            elif message_id == TUSB_MSG_ID_GET_COMMANDS:
                id_type = payload[0]
                if id_type == MARKER_DROPPER:
                    self.send_marker_state(payload[1])
                elif id_type == TORPEDO_SHOOTER:
                    self.send_torpedo_state(payload[1])
                elif id_type == GRIPPER:
                    self.send_gripper_state(payload[1])
            
            elif message_id == TUSB_MSG_ID_MISSION_START:
                self.mission_isr_pressed_cb()      

            elif message_id == TUSB_MSG_ID_DATA_IMU:
                packed_x = payload[:4]
                packed_y = payload[4:8]
                packed_z = payload[8:12]
                packed_w = payload[12:]

                x, y, z, w = self.unpack_float(packed_x), self.unpack_float(packed_y), self.unpack_float(packed_z), self.unpack_float(packed_w)
                self.imu_receive_cb(x, y, z, w)

            elif message_id == TUSB_MSG_ID_CURRENT_TIME_REQUEST:
                    self.send_current_time()
        
        except OSError:
            self.ser = None
            self.software_driver_log("Serial port has been disconnected. Pls connect again.")
            return

    def validate_message(self, message_id, payload_length, payload, checksum):
        validation_checksum = (message_id ^ payload_length)
        for byte in payload:
            validation_checksum ^= byte
        
        return (checksum == validation_checksum)
   
    def unpack_float(self, packed: bytes) -> float:
        if len(packed) != 4:
            raise ValueError("Packed data must be exactly 4 bytes long")

        reconstructed = (
            (packed[0] << 21) |
            (packed[1] << 14) |
            (packed[2] << 7)  |
            (packed[3] << 0)
        )

        reconstructed <<= 4

        return struct.unpack('<f', struct.pack('<I', reconstructed))[0]

    def scale_pwm(self, pwm):
        return (pwm - 1100) // 4

    def send_thruster_pwms(self, pwms):
        print("sending lesssGO")
        pwm_array = []
        for pwm in pwms:
            if pwm > 1900 or pwm < 1100:
                self.software_driver_log("Thruster PWM must be between 1100 and 1900 (message not sent)")
                print("Thruster PWM must be between 1100 and 1900 (message not sent)")
                return
        
            pwm_byte = self.scale_pwm(pwm)
            pwm_array.append(pwm_byte)
            
        self.send_message(TUSB_MSG_ID_THRUSTER_SEND_PWMS, pwm_array)
     
    def send_thruster_individual_pwm(self, thruster_num, pwm):
        if pwm > 1900 or pwm < 1100:
            self.software_driver_log("Thruster PWM must be between 1100 and 1900 (message not sent)")
            print("Thruster PWM must be between 1100 and 1900 (message not sent)")
            return
        
        pwm_byte = self.scale_pwm(pwm)
        self.send_message(TUSB_MSG_ID_THRUSTER_SEND_INDIVIDUAL_PWM, [thruster_num, pwm_byte])

    def soft_kill(self):
        self.send_message(TUSB_MSG_ID_THRUSTER_KILL_UNKILL, [SOFT_KILL])

    def unkill(self):
        self.send_message(TUSB_MSG_ID_THRUSTER_KILL_UNKILL, [UNKILL])

    def set_thruster_pwm_max(self, pwm):
        if pwm > 1900 or pwm < 1100:
            self.software_driver_log("Thruster PWM must be between 1100 and 1900 (message not sent)")
            print("Thruster PWM must be between 1100 and 1900 (message not sent)")
            return
        
        pwm_byte = self.scale_pwm(pwm)
        self.send_message(TUSB_MSG_ID_THRUSTER_CHANGE_LIMITS, [CHANGE_THRUSTER_MAX, pwm_byte])
  
    def set_thruster_pwm_min(self, pwm):
        if pwm > 1900 or pwm < 1100:
            self.software_driver_log("Thruster PWM must be between 1100 and 1900 (message not sent)")
            print("Thruster PWM must be between 1100 and 1900 (message not sent)")
            return
        
        pwm_byte = self.scale_pwm(pwm)
        self.send_message(TUSB_MSG_ID_THRUSTER_CHANGE_LIMITS, [CHANGE_THRUSTER_MIN, pwm_byte])

    def set_thuster_pwm_timeout(self, timeout):
        if timeout > 200 or timeout < 0:
            self.software_driver_log("PWM timeout must be between an integer from 0 to 255 seconds (message not sent)")
            print("PWM timeout must be between an integer from 0 to 255 seconds (message not sent)")
            return
        self.send_message(TUSB_MSG_ID_THRUSTER_CHANGE_PWM_TIMEOUT, [timeout])

    def drop_marker(self):
        self.send_message(TUSB_MSG_ID_SERVO_ACTUATE, [MARKER_DROPPER])

    def shoot_torpedo(self):
        self.send_message(TUSB_MSG_ID_SERVO_ACTUATE, [TORPEDO_SHOOTER])

    def grip(self):
        self.send_message(TUSB_MSG_ID_SERVO_ACTUATE, [GRIPPER])

    def reset_marker_dropper(self):
        self.send_message(TUSB_MSG_ID_SERVO_RESET, [MARKER_DROPPER])

    def reset_torpedo_shooter(self):
        self.send_message(TUSB_MSG_ID_SERVO_RESET, [TORPEDO_SHOOTER])

    def reset_gripper(self):
        self.send_message(TUSB_MSG_ID_SERVO_RESET, [GRIPPER])

    def send_servo_individual_pwm(self, servo_type, pwm):
        if pwm > 2400 or pwm < 700:
            self.software_driver_log("Servo PWM must be between 700 and 2400 (message not sent)")
            print("Thruster PWM must be between 700 and 2400 (message not sent)")
            return
        pwm_lsb = pwm % 64
        pwm_msb = pwm // 64
        self.send_message(TUSB_MSG_ID_SERVO_SEND_CUSTOM_PWM, [servo_type, pwm_msb, pwm_lsb])

    def process_integer_to_bytes(self, n):
        upper_28_bits = (n >> 4)  
        lower_4_bits = n & 0x0F 

        result_bytes = []
        for i in range(4):
            block = (upper_28_bits >> (21 - (i * 7))) & 0x7F 
            result_bytes.append(block)

        byte_5 = lower_4_bits
        result_bytes.append(byte_5)

        return bytes(result_bytes)

    def send_current_time(self):
        current_time = list(self.process_integer_to_bytes(int(time.time())))
        self.send_message(TUSB_MSG_ID_UNIX_TIME, current_time)

    def send_auv_state(self, state):
        self.send_message(TUSB_MSG_ID_AUV_STATE, [state])

    def verbose_on(self):
        self.send_message(TUSB_MSG_ID_TOGGLE, [TOGGLE_ID_VERBOSE, 1])
    
    def verbose_off(self):
        self.send_message(TUSB_MSG_ID_TOGGLE, [TOGGLE_ID_VERBOSE, 0])
 
    def debug_on(self):
        self.send_message(TUSB_MSG_ID_TOGGLE, [TOGGLE_ID_DEBUG, 1])
 
    def debug_off(self):
        self.send_message(TUSB_MSG_ID_TOGGLE, [TOGGLE_ID_DEBUG, 0])
 
    def power_on(self, device):
        self.send_message(TUSB_MSG_ID_TOGGLE, [self.toggle_mappings[device], 1])
     
    def power_off(self, device):
        self.send_message(TUSB_MSG_ID_TOGGLE, [self.toggle_mappings[device], 0])

    def get_data_from_stack(self, data_type):
        self.send_message(TUSB_MSG_ID_GET_COMMANDS, [data_type])

    def get_marker_state(self):
        self.get_data_from_stack(GET_COMMAND_ID_MARKER_STATE)

    def get_torpedo_state(self):
        self.get_data_from_stack(GET_COMMAND_ID_TORPEDO_STATE)

    def get_gripper_state(self):
        self.get_data_from_stack(GET_COMMAND_ID_GRIPPER_STATE)

    def send_marker_state(self, state):
        self.send_to_router(self.ALL, "marker_state", state)

    def send_torpedo_state(self, state):
        self.send_to_router(self.ALL, "torpedo_state", state)

    def send_gripper_state(self, state):
        self.send_to_router(self.ALL, "gripper_state", state)

    def reset_imu(self):
        self.send_message(TUSB_MSG_ID_RESET_IMU, [1])

# think about adding a counter and flush

if __name__=="__main__":#
    print("-------SETTING UP USB DRIVER-------")
    port = sys.argv[1] if len(sys.argv) > 1 else None
    usb_driver=USBDriver(port)
    usb_driver.run()
