#from .constants import controller
from .oldRS_constants import controller

def get_params():
    num_thrust=controller['thrust_position']['num_thrust']
    thruster_names=["surge_top", "surge_left", "surge_right", "sway_back", "sway_front", "heave_back", "heave_left", "heave_right"]

    thrust_pose=[]
    thrust_orien=[]
    surge_power_factors=[]
    surge_backward_ratios=[]
    sway_power_factors=[]
    sway_backward_ratios=[]
    heave_power_factors=[]
    heave_backward_ratios=[]
    rpy_power_factors=[]
    rpy_backward_ratios=[]

    max_front_forces=controller['thrust_position']['max_front_force']
    max_back_forces=controller['thrust_position']['max_back_force']
    max_pwm = controller['thrust_position']['max_pwm']
    min_pwm = controller['thrust_position']['min_pwm']
    zero_pwm = controller['thrust_position']['zero_pwm']
    gamma_front = controller['thrust_position']['gamma_front']
    gamma_back = controller['thrust_position']['gamma_back']

    com_pose = []
    com_pose.append(controller['thrust_position']['COM']['x'])
    com_pose.append(controller['thrust_position']['COM']['y'])
    com_pose.append(controller['thrust_position']['COM']['z'])

    for thruster_name in thruster_names: 
        cur_thrust_x = controller['thrust_position'][f'{thruster_name}']['x']
        cur_thrust_y = controller['thrust_position'][f'{thruster_name}']['y']
        cur_thrust_z = controller['thrust_position'][f'{thruster_name}']['z']
        cur_thrust_roll = controller['thrust_position'][f'{thruster_name}']['roll']
        cur_thrust_pitch = controller['thrust_position'][f'{thruster_name}']['pitch']
        cur_thrust_yaw = controller['thrust_position'][f'{thruster_name}']['yaw']
        cur_surge_power_factor = controller['surge_CEM'][f'{thruster_name}']['power_factor']
        cur_surge_backward_factor = controller['surge_CEM'][f'{thruster_name}']['power_factor']
        cur_sway_power_factor = controller['sway_CEM'][f'{thruster_name}']['power_factor']
        cur_sway_backward_factor = controller['sway_CEM'][f'{thruster_name}']['power_factor']
        cur_heave_power_factor = controller['heave_CEM'][f'{thruster_name}']['power_factor']
        cur_heave_backward_factor = controller['heave_CEM'][f'{thruster_name}']['power_factor']
        cur_rpy_power_factor = controller['rpy_CEM'][f'{thruster_name}']['power_factor']
        cur_rpy_backward_factor = controller['rpy_CEM'][f'{thruster_name}']['power_factor']

        thrust_pose.append([cur_thrust_x, cur_thrust_y, cur_thrust_z])
        thrust_orien.append([cur_thrust_roll, cur_thrust_pitch, cur_thrust_yaw])
        surge_power_factors.append(cur_surge_power_factor)
        surge_backward_ratios.append(cur_surge_backward_factor)
        sway_power_factors.append(cur_sway_power_factor)
        sway_backward_ratios.append(cur_sway_backward_factor)
        heave_power_factors.append(cur_heave_power_factor)
        heave_backward_ratios.append(cur_heave_backward_factor)
        rpy_power_factors.append(cur_rpy_power_factor)
        rpy_backward_ratios.append(cur_rpy_backward_factor)

        power_factors = [surge_power_factors, sway_power_factors, heave_power_factors, rpy_power_factors]
        backward_ratios = [surge_backward_ratios, sway_backward_ratios, heave_backward_ratios, rpy_backward_ratios]

    return (num_thrust, com_pose, thrust_pose, thrust_orien, power_factors, backward_ratios, max_front_forces, max_back_forces, max_pwm, min_pwm, zero_pwm)

def get_control_law_params():
    DOFs = ['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw']

    Kp=[]
    Ki=[]
    Kd=[]
    maxBack=[]
    maxFront=[]

    for dof_name in DOFs:
        Kp.append(controller['pid_constants'][f'{dof_name}']['Kp'])
        Ki.append(controller['pid_constants'][f'{dof_name}']['Ki'])
        Kd.append(controller['pid_constants'][f'{dof_name}']['Kd'])
        maxBack.append(controller['pid_constants'][f'{dof_name}']['maxBack'])
        maxFront.append(controller['pid_constants'][f'{dof_name}']['maxFront'])
        
    return(Kp, Ki, Kd, maxBack, maxFront)
