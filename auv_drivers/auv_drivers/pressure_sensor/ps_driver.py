#!/usr/bin/env python3
import time
import serial

class PsDriver:
    def __init__(self, port="/dev/ttyACM1", baudrate=9600, timeout=0.1):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.depth=0

    def get_depth(self):
        return self.depth
    
    def parse_data(self, raw_data):
        depth=None
        data_string=raw_data.decode('utf-8', 'replace').strip()
        if (len(data_string)>5) and (data_string[:5]=='Depth'):
            string_split=data_string.split(" ")
            if len(string_split)==3:
                depth=float(string_split[1])
        return depth

    def read_data(self):
        depth_data=None
        while not depth_data:
            raw_data=self.ser.read_until(b'\r\n')
            depth_data=self.parse_data(raw_data)
        self.depth=(depth_data/18)


if __name__=="__main__":
    ps_driver=PsDriver()
    print("Using port:", ps_driver.port)

    while True:
        ps_driver.read_data()
        depth=ps_driver.get_depth()
        print("Depth:", depth)

