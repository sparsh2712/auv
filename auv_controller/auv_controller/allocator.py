import numpy as np
import pandas as pd
import bisect
import os
import time
from auv_msgs.msg import PwmData
import rclpy
from rclpy.node import Node

KGF_TO_N=9.806650

# Let the thust_orien be angles between x,y,z axes respectively
# CEM: Control Effectiveness Matrix
class Allocator:
    def __init__(self, num_thrust, com_pose, thrust_pose, thrust_orien,
                 power_factors, backward_ratios, max_front_force,
                 max_back_force, max_pwm, min_pwm, zero_pwm):
       
        thruster_names=["surge_top", "surge_left", "surge_right", "sway_back", "sway_front", "heave_back", "heave_left", "heave_right"]

        self.num_thrust=num_thrust      # Number of thrusters
        self.com_pose=np.array(com_pose)
        self.thrust_pose=np.array(thrust_pose)-self.com_pose      # Thruster positions (x,y,z)
        self.thrust_orien=np.array(thrust_orien)        # Thruster orientations (angles with x,y,z axes; roll, pitch, yaw)

        self.power_factors=power_factors
        self.backward_ratios=backward_ratios
        self.max_front_force=max_front_force
        self.max_back_force=max_back_force
        self.max_pwm=max_pwm
        self.min_pwm=min_pwm
        self.zero_pwm=zero_pwm

        # A list of 4 CEMs:
        #   CEM[0] --> surge
        #   CEM[1] --> sway
        #   CEM[2] --> heave
        #   CEM[3] --> rpy 
        self.CEM = []
        self.CEM_inv = []

        for CEM_index in range(4):
            force_vecs=[]
            for i in range(num_thrust):
                pos=self.thrust_pose[i]     # Thruster position
                thrust_force=np.cos(self.thrust_orien[i])       # Thruster force vector, represents force distribution
                
                thrust_force *= self.power_factors[CEM_index][i]
                thrust_torque=np.cross(pos, thrust_force)       # Thruster torque vector, represents torque distribution
                force_vec=np.hstack((
                    thrust_force, thrust_torque))       # Thruster forces stacked to give a 6-dimensional vector
                force_vecs.append(force_vec)

            self.CEM.append(np.array(force_vecs).transpose())      # Control Effectiveness matrix
            self.CEM[CEM_index][np.abs(self.CEM[CEM_index])<0.00001]=0        # Done so that unused thrusters don't receive power
            self.CEM_inv.append(np.linalg.pinv(self.CEM[CEM_index]))       # Pseudo-inverse matrix of control effectiveness matrix

        #script_directory = os.path.dirname(os.path.abspath(__file__))
        #file_path = os.path.join(script_directory, 'tuning_data', 'force_to_pwm.xlsx')
        file_path = "~/ros2_ws/src/Robosub_ROS2_2024/auv_controller/auv_controller/tuning_data/force_to_pwm.xlsx"
        
        df=pd.read_excel(file_path, sheet_name="16 V")
        pwms_list=list(df[" PWM (µs)"])
        self.forces_list=list(df[" Force (Kg f)"]*KGF_TO_N)

        map_series=pd.Series(pwms_list, index=self.forces_list)
        self.force_pwm_map=map_series.to_dict()

    def find_closest_force(self, target_force):
        index = bisect.bisect_left(self.forces_list, target_force)
        if index == 0:
            return self.forces_list[0]
        if index == len(self.forces_list):
            return self.forces_list[index -1]
        before = self.forces_list[index - 1]
        after = self.forces_list[index]
        if after - target_force < target_force - before:
            return after
        else:
            return before
        



    def adjust_backward(self, thrust_forces, backward_ratios):
        for i in range(len(thrust_forces)):
            if thrust_forces[i] < 0:
                thrust_forces[i] *= backward_ratios[i]
        return thrust_forces
    
    def force_to_thrust(self, body_forces):
        split_body_forces = []
        split_body_forces.append([body_forces[0], 0, 0, 0, 0, 0])
        split_body_forces.append([0, body_forces[1], 0, 0, 0, 0])
        split_body_forces.append([0, 0, body_forces[2], 0, 0, 0])
        split_body_forces.append([0, 0, 0, body_forces[3], body_forces[4], body_forces[5]])
        
        thrust_forces = np.zeros(8, dtype=float)
        for CEM_index in range(4):
            split_thruster_forces = self.CEM_inv[CEM_index].dot(split_body_forces[CEM_index])
            split_thruster_forces = self.adjust_backward(split_thruster_forces,
                                                         self.backward_ratios[CEM_index])
            thrust_forces += split_thruster_forces

        thrust_max_ratios=[]
        for i in range(len(thrust_forces)):
            if thrust_forces[i]>=0:
                cur_ratio=abs(thrust_forces[i]/self.max_front_force)
            else:
                cur_ratio=abs(thrust_forces[i]/self.max_back_force)
            thrust_max_ratios.append(cur_ratio)
        
        cap_ratio=max(1, max(thrust_max_ratios))
        thrust_capped=thrust_forces/cap_ratio       # Capping thruster forces with maximum 
        return thrust_capped

    def thrust_to_pwm(self, thrust_forces):
        pwms=[]
        for i in range(len(thrust_forces)):
            thrust_force=thrust_forces[i]
            closest_force=self.find_closest_force(thrust_force)
            mapped_pwm=self.force_pwm_map[closest_force]
            pwms.append(mapped_pwm)
        return pwms
       
    def thrust_to_force(self, thrust_forces):
        body_forces=self.CEM.dot(thrust_forces)
        return body_forces


