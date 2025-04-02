import time
import serial
import rclpy
import os

class CanDriver:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.1):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.write(b'C\r')
        self.ser.write(b'S6\r')
        self.ser.write(b'O\r')
        self.ser.write(b't040100\r')
        self.power_bits=[True,True,True,True,True,True,True,True]
        self.counter=0 
        while True:
            data=self.ser.read_until(b'\r')
            print(data)
            if data==b'':
                break

    """
    CAN Thruster mappings:
        surge_top: 0 -> 1 -> heave_right (1)
        sway_back: 1 -> 2 -> sway_back (2)
        heave_back: 2 
        heave_right: 3
        heave_left: 4
        surge_right: 5 -> 5 -> surge_right(5)
        surge_left: 6
        sway_front: 7
    """
    def send_pwm_message(self, thruster_pwms):
        pwm_list=[]

        # DO NOT CHANGE THIS MAPPING HERE. CHANGE IT IN ALLOCATOR
        # LEST YOU WANT TO MESS ALLOCATOR TUNING UP

        pwm_list.append(3000 - thruster_pwms["sway_front"])
        pwm_list.append(thruster_pwms["surge_top"])
        pwm_list.append(thruster_pwms["heave_right"])
        pwm_list.append(3000 - thruster_pwms["surge_right"])
        pwm_list.append(3000 - thruster_pwms["sway_back"])
        pwm_list.append(thruster_pwms["heave_left"])
        pwm_list.append(thruster_pwms["surge_left"])
        pwm_list.append(thruster_pwms["heave_back"])

        pwm_list_1=pwm_list[:4]
        pwm_list_2=pwm_list[4:]

        pwm1_str='t0518'
        for pwm in pwm_list_1:
            pwm_hex=hex(pwm)[2:].zfill(4)
            pwm1_str+=pwm_hex
        pwm1_str+='\r'
        pwm1_msg=pwm1_str.encode()

        pwm2_str='t0528'
        for pwm in pwm_list_2:
            pwm_hex=hex(pwm)[2:].zfill(4)
            pwm2_str+=pwm_hex
        pwm2_str+='\r'
        pwm2_msg=pwm2_str.encode()
        
        self.counter+=1
        
        if self.counter>50:
            print("Flushing can")
            self.counter=0
            #self.ser.flush()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.flush_can()
            time.sleep(0.1)

        self.ser.write(pwm1_msg)
        self.ser.write(pwm2_msg)
    
    def send_actuator_message(self, torpedo_bits, marker_dropper_bits, gripper_bits):
        actuator_str='t0558'
        actuator_str+=torpedo_bits
        actuator_str+=marker_dropper_bits
        actuator_str+=gripper_bits
        actuator_str+='0000000000'
        actuator_str+='\r'

        actuator_str=str(actuator_str)
        actuator_msg=actuator_str.encode()

        self.ser.write(actuator_msg)


    def flush_can(self):
        self.ser.write(b'C\r')
        self.ser.write(b'S6\r')
        self.ser.write(b'O\r')
        self.ser.write(b't040100\r')
        print("can flushed")
    

    def flush_can_reset(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.flush_can()
        time.sleep(0.1)

    def read_message(self):
        data=self.ser.read_until(b'\r')
        return data


    def close(self):
        self.ser.write(b'C\r')
        self.ser.close()


    def can_kill(self):
        self.ser.write(b"t05A100\r")
        pwm_msg={}
        pwm_msg["surge_top"]=1500
        pwm_msg["surge_left"]=1500
        pwm_msg["surge_right"]=1500
        pwm_msg["sway_back"]=1500
        pwm_msg["sway_front"]=1500
        pwm_msg["heave_back"]=1500
        pwm_msg["heave_left"]=1500
        pwm_msg["heave_right"]=1500
        self.send_pwm_message(pwm_msg)

    def can_unkill(self):
        self.ser.write(b"t05A101\r")
        self.ser.write(b"t051805dc05dc05dc05dc")
        self.ser.write(b"t052805dc05dc05dc05dc")

    def dvl_on(self):
        self.power_bits[3]=True #index was 4 before
        self.power_write()


    def dvl_off(self):
        self.power_bits[3]=False #index was 4 earlier
        self.power_write()

    def basler_on(self):
        self.power_bits[1]=True
        self.power_write()

    def basler_off(self):
        self.power_bits[1]=False
        self.power_write()

    def power_write(self):
        self.power_str=''
        for a in self.power_bits:
            if a:
                self.power_str+='11'
            else:
                self.power_str+='00'
        self.ser.write(bytes('t0618'+self.power_str+'\r','utf-8'))

if __name__=="__main__":

    print("-------SETTING UP CAN DRIVER-------")
    #os.system('sudo chmod a+rwx /dev/tty*')
    can_driver=CanDriver("/dev/ttyUSB0")
    
    time.sleep(60)
    
    thruster_pwms={
        "surge_top": 1460,
        "surge_left": 1528,
        "surge_right": 1532,
        "sway_back": 1528,
        "sway_front": 1528,
        "heave_back": 1630,
        "heave_left": 1580,
        "heave_right": 1610
    }
    can_driver.send_pwm_message(thruster_pwms)
    time.sleep(9)
    thruster_pwms={
        "surge_top": 1500,
        "surge_left": 1500,
        "surge_right": 1500,
        "sway_back": 1500,
        "sway_front": 1500,
        "heave_back": 1500,
        "heave_left": 1500,
        "heave_right": 1500
    }
    can_driver.send_pwm_message(thruster_pwms)
    time.sleep(0.1)

    thruster_pwms={
        "surge_top": 1584,
        "surge_left": 1600,
        "surge_right": 1600,
        "sway_back": 1440,
        "sway_front": 1456,
        "heave_back": 1580,
        "heave_left": 1573,
        "heave_right": 1545
    }
    #time.sleep(60) 
    can_driver.send_pwm_message(thruster_pwms)
    time.sleep(15)
    

    thruster_pwms={
        "surge_top": 1500,
        "surge_left": 1500,
        "surge_right": 1500,
        "sway_back": 1500,
        "sway_front": 1500,
        "heave_back": 1500,
        "heave_left": 1500,
        "heave_right": 1500
    }
    print("-------SHUTTING OFF THRUSTERS-------")
    can_driver.send_pwm_message(thruster_pwms)    
