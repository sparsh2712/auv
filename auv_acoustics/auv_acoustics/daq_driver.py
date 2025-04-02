#this file is for the implementation of DAQ driver class 
#Class DAQ_driver 
#All functions which should not be called from outside (pseudo private functions) start with _
#Lock thread while collecting data from daq (will be handled in the main acoustic class)
#methods: 
#sample_data - collect data for 2 seconds add it to a queue (length 800,000) to have a buffer of one packet 
#_get_sample - get data from hardware 
#stop 
#start 

import nidaqmx
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory as get_pack

class DAQ:
    def __init__(self, params):
        self.num_hydrophones = params[0]
        self.sampling_freq = params[1]
        self.signal_len = params[2]
        self.sampled_data = None
        
    def start_sampling(self):
        print("sampling data...")
        with nidaqmx.Task() as task:
            for channel in range(self.num_hydrophones):
                task.ai_channels.add_ai_voltage_chan(f"cDAQ1Mod1/ai{channel}")
            task.timing.cfg_samp_clk_timing(rate=self.sampling_freq, \
                sample_mode=nidaqmx.constants.AcquisitionType.FINITE, samps_per_chan=self.signal_len)
            self.sampled_data = task.read(number_of_samples_per_channel = self.signal_len)
        print("sampling done!!")