#import rospy

def get_models():
    models = {'Face': 'CV','bin': 'CV', 'buoy': 'CV'}
    return models

def get_vision_params():
	front_camera_config = {'img_height': 720, 'img_width': 1280} #rospy.get_param("/vision/front_camera_config")
	bottom_camera_config = {'img_height': 720, 'img_width': 1280} #rospy.get_param("/vision/bottom_camera_config")
	dvl_to_front_cam_vector = {'x': 0, 'y': 0, 'z': 0}#{'x': 55, 'y': 7, 'z': -7} #rospy.get_param("/vision/dvl_to_camera_vector/front")
	dvl_to_bottom_cam_vector = {'x': 13, 'y': 9, 'z': -3} #rospy.get_param("/vision/dvl_to_camera_vector/bottom")
	objects = [
    {
        "object_id": 0,
        "object_name": "Face",
        "object_length": 20,
        "object_height": 18,
        "agreement_threshold": 20,
        "min_confidence": 0.0
    },
    {
        "object_id": 2,
        "object_name": "Buoy",
        "object_length": 15,
        "object_height": 15,
        "agreement_threshold": 20,
        "min_confidence": 0.4
    },
    {
        "object_id": 3,
        "object_name": "Torpedo_1",
        "object_length": 7.5,
        "object_height": 7.5,
        "agreement_threshold": 20,
        "min_confidence": 0.4
    },
    {
        "object_id": 4,
        "object_name": "Torpedo_2",
        "object_length": 20,
        "object_height": 20,
        "agreement_threshold": 20,
        "min_confidence": 0.4
    },
    {
        "object_id": 5,
        "object_name": "Torpedo_3",
        "object_length": 26,
        "object_height": 26,
        "agreement_threshold": 20,
        "min_confidence": 0.4
    },
    {
        "object_id": 6,
        "object_name": "Torpedo_4",
        "object_length": 30,
        "object_height": 30,
        "agreement_threshold": 20,
        "min_confidence": 0.4
    },
    {
        "object_id": 7,
        "object_name": "Torpedo_Flex",
        "object_length": 60,
        "object_height": 60,
        "agreement_threshold": 50,
        "min_confidence": 0.4
    },
    {
        "object_id": 8,
        "object_name": "Gate_Blue_Side",
        "object_length": 30,
        "object_height": 30,
        "agreement_threshold": 40,
        "min_confidence": 0.4
    },
    {
        "object_id": 9,
        "object_name": "Gate_Red_Side",
        "object_length": 30,
        "object_height": 30,
        "agreement_threshold": 40,
        "min_confidence": 0.4
    },
    {
        "object_id": 10,
        "object_name": "Bin",
        "object_length": 180,
        "object_height": 120,
        "agreement_threshold": 30,  #20
        "min_confidence": 0.4
    },
    {
        "object_id": 11,
        "object_name": "Blue_Box",
        "object_length": 30,
        "object_height": 30,
        "agreement_threshold": 20,
        "min_confidence": 0.4
    },
    {
        "object_id": 12,
        "object_name": "Red_Box",
        "object_length": 30,
        "object_height": 30,
        "agreement_threshold": 20,
        "min_confidence": 0.4
    }
]  #rospy.get_param("/vision/objects")
	return (front_camera_config, bottom_camera_config,
	 dvl_to_front_cam_vector, dvl_to_bottom_cam_vector, objects)

def get_task_details():
	task_details= task_details = [
    {
        "task_name": "Bin",
        "objects": [
            {
                "name": "Bin",
                "rel_vector_from_task": {"x": 15, "y": 0, "z": 0},
                "priority": 1
            }
        ]
    },
    {
        "task_name": "Buoy",
        "objects": [
            {
                "name": "Buoy",
                "rel_vector_from_task": {"x": 0, "y": 0, "z": 0},
                "priority": 1
            }
        ]
    },
    {
        "task_name": "TempBuoy",
        "objects": [
            {
                "name": "Buoy",
                "rel_vector_from_task": {"x": 0, "y": 0, "z": 0},
                "priority": 1
            }
        ]
    },
    {
        "task_name": "TorpedoSafe",
        "objects": [
            {
                "name": "Torpedo_1",
                "rel_vector_from_task": {"x": 0, "y": 0, "z": 0},
                "priority": 1
            }
        ]
    },
    {
        "task_name": "Torpedo",
        "objects": [
            {
                "name": "Torpedo_1",
                "rel_vector_from_task": {"x": 0, "y": 0, "z": 0},
                "priority": 1
            }
        ]
    },
    {
        "task_name": "TorpedoOld",
        "objects": [
            {
                "name": "Torpedo_1",
                "rel_vector_from_task": {"x": 0, "y": 40, "z": 0},
                "priority": 1
            },
            {
                "name": "Torpedo_2",
                "rel_vector_from_task": {"x": 0, "y": -40, "z": 0},
                "priority": 2
            },
            {
                "name": "Torpedo_3",
                "rel_vector_from_task": {"x": 0, "y": -40, "z": 0},
                "priority": 3
            },
            {
                "name": "Torpedo_4",
                "rel_vector_from_task": {"x": 0, "y": 40, "z": 0},
                "priority": 4
            }
        ]
    },
    {
        "task_name": "TorpedoFlex",
        "objects": [
            {
                "name": "Torpedo_Flex",
                "rel_vector_from_task": {"x": 0, "y": 0, "z": 0}
            }
        ]
    },
    {
        "task_name": "Gate",
        "objects": [
            {
                "name": "Gate_Red_Side",
                "rel_vector_from_task": {"x": 0, "y": 0, "z": -100},
                "priority": 1
            }
        ]
    }
]  #rospy.get_param("/vision/tasks")
	return task_details

if __name__=="__main__":
	vision_params=get_vision_params()
	task_details=get_task_details()
	print(vision_params)
	print(task_details)
