controller={
    'thrust_position':{ 
        'num_thrust': 8,
        'max_front_force': 43.605,
        'max_back_force': 34.187,
        'max_pwm': 1650,
        'min_pwm': 1350,
        'zero_pwm': 1500,
        'gamma_front': 1,
        'gamma_back': 1,
        'COM': {
            'x': -0.429, #-0.429
            'y': -0.175,
            'z': -0.03, #-0.025
        },
        'surge_top':{
            'x': -0.776,
            'y': -0.126,
            'z': -0.204,
            'roll': 0,
            'pitch': -1.57,
            'yaw': 1.57,
        },
        'surge_left':{
            'x': -0.644,
            'y': -0.38,
            'z': 0.052,
            'roll': 0,
            'pitch': -1.57,
            'yaw': 1.57,
        },
        'surge_right':{
            'x': -0.644,
            'y': 0.127,
            'z': 0.052,
            'roll': 0,
            'pitch': -1.57,
            'yaw': 1.57,
        },
        'sway_back':{
            'x': -0.826,
            'y': -0.105,
            'z': 0.032,
            'roll': 1.57,
            'pitch': 0,
            'yaw': 1.57,
        },
        'sway_front': {
            'x': -0.027,
            'y': -0.129,
            'z': -0.09,
            'roll': 1.57,
            'pitch': 0,
            'yaw': 1.57,
        },
        'heave_left':{
            'x': -0.102,
            'y': -0.38,
            'z': -0.072,
            'roll': -1.57,
            'pitch': 1.57,
            'yaw': 0,  
        },
        'heave_right':{
            'x': -0.102,
            'y': 0.127,
            'z': -0.073,
            'roll': -1.57,
            'pitch': 1.57,
            'yaw': 0,
        },
        'heave_back':{
            'x': -0.912,
            'y': -0.126,
            'z': -0.0925,
            'roll': -1.57,
            'pitch': 1.57,
            'yaw': 0,
        },
    },
    'pid_constants': {
        'surge': {
            'Kp': 0.5,
            'Ki': 0,
            'Kd': 0.7,
            'maxBack': -70,
            'maxFront': 70,
        },
        'sway': {
            'Kp': 0.3,
            'Ki': 0,
            'Kd': 0,
            'maxBack': -70,
            'maxFront': 70,
        },
        'heave': {
            'Kp': 0.6,
            'Ki': 0.012,
            'Kd': 0.09,
            'maxBack': -70,
            'maxFront': 70,
        },
        'roll':{
            'Kp': 0.1,
            'Ki':0,
            'Kd': 0.01,
            'maxBack': -20,
            'maxFront': 20,
        },
        'pitch':{
            'Kp': 0.2,
            'Ki': 0,
            'Kd': 0.02,
            'maxBack': -40,
            'maxFront': 40,
        },
        'yaw':{
            'Kp': 0.3,
            'Ki': 0,
            'Kd': 0.1,
            'maxBack': -40,
            'maxFront': 40,
        },
    },
    'heave_CEM': {
        'surge_top':{
            'power_factor': 0.85,
            'backward_ratio': 1.000,
        },
        'surge_left':{
            'power_factor': 0.99,
            'backward_ratio': 1.000,
        },
        'surge_right': {
            'power_factor': 0.941,
            'backward_ratio': 1.000,
        },
        'sway_back':{
            'power_factor': 0.320,
            'backward_ratio': 1.000,
        },
        'sway_front':{
            'power_factor': 0.950,
            'backward_ratio': 1.000,
        },
        'heave_left':{
            'power_factor': 1.6,
            'backward_ratio': 0.800,
        },
        'heave_right':{
            'power_factor': 0.6,
            'backward_ratio': 1.000,
        },
        'heave_back':{
            'power_factor': 0.8,
            'backward_ratio': 1.3,
        },
    },
    'rpy_CEM':{
        'surge_top': {
            'power_factor': 0.85,
            'backward_ratio': 1.000,
        },
        'surge_left': {
            'power_factor': 0.99,
            'backward_ratio': 1.000,
        },
        'surge_right': {
            'power_factor': 0.941,
            'backward_ratio': 1.000,
        },
        'sway_back': {
            'power_factor': 0.320,
            'backward_ratio': 1.000,
        },
        'sway_front': {
            'power_factor': 0.950,
            'backward_ratio': 1.000,
        },
        'heave_left': {
            'power_factor': 1.000,
            'backward_ratio': 1.000,
        },
        'heave_right': {
            'power_factor': 0.883,
            'backward_ratio': 1.000,
        },
        'heave_back': {
            'power_factor': 0.984,
            'backward_ratio': 1.000,
        },  

    }, 
    'surge_CEM':{
        'surge_top': {
            'power_factor': 0.55,
            'backward_ratio': 1.100,
        },
        'surge_left': {
            'power_factor': 1.25, #0.2 #0.3 #0.1
            'backward_ratio': 0.0 # 1.150,
        },
        'surge_right': {
            'power_factor': 0.65, #0.6 #1.2 #1.0
            'backward_ratio': 0.0 #0.950,
        },
        'sway_back': {
            'power_factor': 0.440,
            'backward_ratio': 1.000,
        },
        'sway_front': {
            'power_factor': 0.950,
            'backward_ratio': 1.000,
        },
        'heave_left': {
            'power_factor': 0.95, #0.95
            'backward_ratio': 1.000,
        },
        'heave_right': {
            'power_factor': 0.87, #0.87
            'backward_ratio': 1.000,
        },
        'heave_back': {
            'power_factor': 0.984,
            'backward_ratio': 1.000,
        },

    },
    'sway_CEM':{
        'surge_top': {
            'power_factor': 0.85,
            'backward_ratio': 1.000,
        },
        'surge_left': {
            'power_factor': 0.99,
            'backward_ratio': 1.000,
        },
        'surge_right': {
            'power_factor': 0.941,
            'backward_ratio': 1.000,
        },
        'sway_back': {
            'power_factor': 0.480,
            'backward_ratio': 1.000,
        },
        'sway_front': {
            'power_factor': 0.950,
            'backward_ratio': 1.000,
        },
        'heave_left': {
            'power_factor': 1.000,
            'backward_ratio': 1.000,
        },
        'heave_right': {
            'power_factor': 0.933,
            'backward_ratio': 1.000,
        },
        'heave_back': {
            'power_factor': 0.104,
            'backward_ratio': 1.000,
        },

    },

}
