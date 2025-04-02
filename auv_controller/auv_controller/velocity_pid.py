import numpy as np
from simple_pid import PID
from scipy.spatial.transform import Rotation

class PIDControl:
    def __init__(self, Kp, Ki, Kd, maxBack, maxFront, use_new_model=False):
        self.use_new_model = use_new_model
        self.pidArray = [PID() for _ in range(6)]
        self.Kp = Kp
        self.Kd = Kd
        for i in range(6):
            self.pidArray[i].setpoint = 0
            self.pidArray[i].tunings = (Kp[i], Ki[i], Kd[i])
            self.pidArray[i].output_limits = (maxBack[i], maxFront[i])
        self.output = np.empty(6, dtype=float)
    
    def compute_forces(self, setpoint, pose, current_vel=None, setpoint_vel=None):
        
        global_pose_error = np.array(pose)[:3] - np.array(setpoint)[:3]
        
        rot = Rotation.from_euler('ZYX', (pose[5], pose[4], pose[3]), degrees=True)
        inv_rot = Rotation.from_euler('XYZ', (-pose[3], -pose[4], -pose[5]), degrees=True)
        local_error = np.zeros(6, dtype=float)
        local_error[:3] = np.matmul(inv_rot.as_matrix(), global_pose_error[:3])
        
        inv_orien_setpoint = Rotation.from_euler('XYZ', (-setpoint[3], -setpoint[4], -setpoint[5]), degrees=True)
        local_error[5], local_error[4], local_error[3] = (inv_orien_setpoint * rot).as_euler('ZYX', degrees=True)
        
        if self.use_new_model and (current_vel is not None) and (setpoint_vel is not None):
          
            for i in [0, 1]:
                pos_error = setpoint[i] - pose[i]
                vel_error = setpoint_vel[i] - current_vel[i]
                self.output[i] = self.Kp[i] * pos_error + self.Kd[i] * vel_error
        else:
           
            self.output[0] = self.pidArray[0](local_error[0])
            self.output[1] = self.pidArray[1](local_error[1])
        
        self.output[2] = self.pidArray[2](local_error[2])
        self.output[3] = self.pidArray[3](local_error[3])
        self.output[4] = self.pidArray[4](local_error[4])
        self.output[5] = self.pidArray[5](local_error[5])
        
        return self.output
