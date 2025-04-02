#this file is for the implementation of PingLocator class 
#Class PingLocator 
#All functions which should not be called from outside (pseudo private functions) start with _
#methods: 
#locate - will handle which startegy to use based on config and call the relevant function 
#_left_right_two_hydrophone_startegy  - Produce pseudo setpoints on the basis of which peak is recieved first 
#_exact_location_three_hydrophone_startegy - Produce psuedo setpoints based on three hydrophones 
#any other strategy 


#                                         *(2)

#orientation of hydrophone ----- *(1)     *(0)

import numpy as np 
from scipy.optimize import minimize

class PingLocator():
    def __init__(self):
        # all params to be loaded from a config 
        self.number_of_hydrophones = 3 #loaded from config 
        self.pseudo_pinger_location = [0,0,0,0,0,0]
        self.step_size = 10
        self.speed_of_sound = 1400
        self.hydrophone_pos = np.array([[1,0],[0,0],[1,1]]) #loaded from config 

    def locate(self,time_stamps:np.ndarray)->np.ndarray:
        if len(time_stamps) != self.number_of_hydrophones:
            raise ValueError(f"expected {self.number_of_hydrophones} time stamps but recieved {time_stamps.size}")
        if self.number_of_hydrophones == 3:
            self._three_hydrophone_strategy(time_stamps)
        elif self.number_of_hydrophones == 2:
            self._two_hydrophone_startegy(time_stamps)
        else:
            raise ValueError (f"Invalid number of hydrophones")
        
        return self.pseudo_pinger_location

    def _distance(self, index, pinger_pos):
        return np.sqrt((pinger_pos[0] - self.hydrophone_pos[index][0])**2 + (pinger_pos[1] - self.hydrophone_pos[index][1])**2)
    
    def _objective_function_three_hydrophone(self, pinger_pos, distance_diff):
        x,y = pinger_pos

        distance_0 = self._distance(0, pinger_pos)
        distance_1 = self._distance(1, pinger_pos)
        distance_2 = self._distance(2, pinger_pos)
        return np.abs((distance_1 - distance_0 - distance_diff[1]))+np.abs((distance_2 - distance_0 - distance_diff[2]))

    def _constraints_three_hydrophone(self, pinger_pos, distance_diff):
        sorted_indices = np.argsort(distance_diff)
        distance_min = self._distance(sorted_indices[0], pinger_pos)
        distance_mid = self._distance(sorted_indices[1], pinger_pos)
        distance_max = self._distance(sorted_indices[2], pinger_pos)
        return (distance_max - distance_mid), (distance_mid - distance_min)

    def _get_pinger_pose(self, time_stamps:np.ndarray)->np.ndarray:
        if self.number_of_hydrophones==3: 
            time_diff = time_stamps - time_stamps[0] 
            distance_diff = self.speed_of_sound*time_diff
            print(distance_diff)
            intial_guess = (5, 5)
            constraints = [
            {'type': 'ineq', 'fun': lambda pinger_pos: self._constraints_three_hydrophone(pinger_pos, distance_diff)[0]},
            {'type': 'ineq', 'fun': lambda pinger_pos: self._constraints_three_hydrophone(pinger_pos, distance_diff)[1]}
            ]
            result = minimize(
                fun = self._objective_function_three_hydrophone,
                x0=intial_guess,
                args=(distance_diff),
                constraints=constraints,
            )
        else:
            #logic to get pseudo pinger pose with 2 hydrophones
            pass
        
        print(result)
        if result.success:
            return np.array(result.x[0], result.x[1])
        else: 
            print("failed to locate pinger")
    
    def _three_hydrophone_strategy(self,timestamp):
        pinger_pose = self._get_pinger_pose(timestamp)
        theta = np.arctan(pinger_pose[1]/pinger_pose[0])
        self.pseudo_pinger_location = np.array([self.step_size*np.cos(theta), self.step_size*np.sin(theta), 0, 0, 0, 0])


if __name__ == "__main__":
    obj = PingLocator()
    data= np.load('/Users/sparsh/ros2_ws/src/Robosub_ROS2_2024/sim_data.npy')
    time_stamps = np.array([np.argmax(d)/200000 for d in data])
    pseudo_pinger_location = obj.locate(time_stamps)
    print(pseudo_pinger_location)




            

