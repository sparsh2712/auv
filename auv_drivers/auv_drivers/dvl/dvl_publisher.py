#!/usr/bin/python3
from cgitb import reset
import socket
import json
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.exceptions import ROSInterruptException
from time import sleep
from std_srvs.srv import Empty
from auv_msgs.msg import DVLVel
from auv_msgs.msg import DVLBeam
from auv_msgs.msg import DVLBeamsArr
from auv_msgs.msg import DVLOrient


print("hello")

def connect():
    global dvl_socket, TCP_IP, TCP_PORT
    dvl_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while rclpy.ok():
        sleep(1)
        try:
            dvl_socket.connect((TCP_IP, TCP_PORT))
            break
        except socket.error as err :
            print("######  ERROR : No route to host, DVL might be booting?  ########")
            #Node.get_logger().info("No route to host, DVL might be booting? {}".format(err))
    dvl_socket.settimeout(1)


def calibrate_gyro():
    buffer_size = 4096
    cmd_msg = {"command": "calibrate_gyro"}
    jsonObj  = json.dumps(cmd_msg)
    dvl_socket.send(jsonObj + "\r\n")
    print("Sent the calibrate_gyro message")
    message = dvl_socket.recv(buffer_size).decode()
    print("Received response")
    data = json.loads(message)
    if data["type"]:
        print(data)

def get_config():
    buffer_size = 4096
    cmd_msg = {"command": "get_config"}
    jsonObj  = json.dumps(cmd_msg)
    dvl_socket.send(jsonObj + "\r\n")
    print("Sent the get_config message")
    message = dvl_socket.recv(buffer_size).decode()
    print("Received response")
    data = json.loads(message)
    if data["type"]:
        print(data["success"])

def set_config():
    #Donot call the function, might set wrong values
    buffer_size = 4096
    cmd_msg = {"command": "set_config",
        "parameters":{"speed_of_sound":1481, 
        "mounting_rotation_offset":0.00}}
    jsonObj  = json.dumps(cmd_msg)
    dvl_socket.send(jsonObj + "\r\n")
    print("Sent the set_config message")
    message = dvl_socket.recv(buffer_size).decode()
    print("Received response")
    data = json.loads(message)
    if data["type"]:
        print(data["success"])

class DvlPublisher(Node):
    def __init__(self):
        super().__init__('dvl_publisher')
        global dvl_socket, TCP_IP, TCP_PORT, do_log_raw_data
        self.vel_pub = self.create_publisher(DVLVel, 'dvl/velData', 10)
        self.orient_pub = self.create_publisher(DVLOrient,'dvl/orientData', 10)
        self.beams_pub = self.create_publisher(DVLBeamsArr, 'dvl/beamsData', 10)
        self.reset_service = self.create_service(Empty, 'dvl_a50/reset', self.cbReset)
        self.current_time = None
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip', rclpy.Parameter.Type.STRING),
                ('port', rclpy.Parameter.Type.INTEGER)
            ]
        )
        
        TCP_IP = "192.168.194.95"
        TCP_PORT = 16171
        TCP_IP = self.get_parameter('ip').get_parameter_value().string_value
        TCP_PORT = self.get_parameter('port').get_parameter_value()._integer_value

        connect()

        try:
            self.run()
        except ROSInterruptException:
            dvl_socket.close()
        

    def reset_dead_reckoning(self):
        buffer_size = 4096
        cmd_msg = {"command": "reset_dead_reckoning"}
        jsonObj  = json.dumps(cmd_msg) + "\r\n"
        json_encode = jsonObj.encode('utf-8')
        dvl_socket.send(json_encode)
        print("Sent the reset message")
        message = dvl_socket.recv(buffer_size).decode()
        print("Received response")
        data = json.loads(message)

    def cbReset(self,msg):
        self.reset_dead_reckoning()
        return Empty()
    
    def publish_msg(self,data):
        beams = [DVLBeam(), DVLBeam(), DVLBeam(), DVLBeam()]
        if data["type"] == "position_local":
            DVLorient = DVLOrient()
            print(type(self.current_time))
            print(self.current_time)
            # DVLorient.header.stamp.sec = self.current_time.sec #stamp.sec works but self.current_time.sec does not, same for nsec/nanosec
            # DVLorient.header.stamp.nsec = self.current_time.nanosec
            DVLorient.header.frame_id = "dvl_link_1"
            DVLorient.ts = data["ts"]
            DVLorient.position.x = float(data["x"])
            DVLorient.position.y = float(data["y"])
            DVLorient.position.z = float(data["z"])
            DVLorient.roll = float(data["roll"])
            DVLorient.pitch = float(data["pitch"])
            DVLorient.yaw = float(data["yaw"])
            self.orient_pub.publish(DVLorient)

        elif data["type"] == "velocity":
            DVLvel = DVLVel()
            #DVLvel.header.stamp = self.current_time 
            DVLvel.header.frame_id = "dvl_link_2"
            DVLvel.time = data["time"]
            DVLvel.velocity.x = float(data["vx"])
            DVLvel.velocity.y = float(data["vy"])
            DVLvel.velocity.z = float(data["vz"])
            DVLvel.fom = float(data["fom"])
            DVLvel.altitude = float(data["altitude"])
            DVLvel.velocity_valid = data["velocity_valid"]
            DVLvel.status = data["status"]
            DVLvel.form = data["format"]
            self.vel_pub.publish(DVLvel)

            for i in range(4):
                beams[i].id = data["transducers"][i]["id"]
                beams[i].velocity = float(data["transducers"][i]["velocity"])
                beams[i].distance = float(data["transducers"][i]["distance"])
                beams[i].rssi = data["transducers"][i]["rssi"]
                beams[i].nsd = data["transducers"][i]["nsd"]
                beams[i].valid = data["transducers"][i]["beam_valid"]

            DVLbeams = DVLBeamsArr()
            DVLbeams.beams = beams
            self.beams_pub.publish(DVLbeams)

    def run(self):
        global dvl_socket
        buffer_size = 4096
        message = ""
        while True: 
            self.get_logger().info("before socket.recv")
            try:
                buffer = dvl_socket.recv(buffer_size).decode()
                self.get_logger().info("crossed buffer")
                if not buffer:
                    continue
                message_parts = buffer.split("\r\n")
                message_parts[0] = message + message_parts[0]
                ##############################################
                self.current_time = self.get_clock().now()      #gives correct time but format type : <class 'rclpy.time.Time'>, but header.stamp wants in 'time'
                ##############################################
                for message_part in message_parts[:-1]:
                    try:
                        data = json.loads(message_part)
                        if data["type"] != "response":
                            self.publish_msg(data)
                    except ValueError:
                        print("Could not parse to JSON: " + message_part)
                message = message_parts[-1] if message_parts[-1] else ""
            except:
                self.get_logger().info("DVL socket not connected?")
                sleep(1)
                continue\

def main(args=None):
    rclpy.init(args=args)
    dvl_publisher = DvlPublisher()
    rclpy.spin(dvl_publisher)
    dvl_publisher.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    global dvl_socket, TCP_IP, TCP_PORT, do_log_raw_data
    do_log_raw_data = False
    main()



        
