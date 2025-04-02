#this file is for the implementation of Acoustic class 
#Class Acoustic
#All functions which should not be called from outside (pseudo private functions) start with _
#methods: 
#__init__: Initialises daq_driver, ping_locator, signal_processing class
#services: acoustic_toggle - switch data collection and processing on and off 
#        : get pinger pose 

#acoustic_toggle_func - start or stop data acq
#get_pinger_pose_func - return pseudo pinger_loc
#run - acquire_packet, proess_signal, update_pseudo_pos  

import rclpy
import numpy as np

class Acoustics:

    def __init__(self):
        pass

    def run(self):
        pass

def main():
    pass

if __name__ == "__main__":
    main()