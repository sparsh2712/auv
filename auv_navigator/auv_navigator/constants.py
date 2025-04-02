class NavigatorParams:
    def __init__(self):
        self.xy_tolerance = 20.0
        self.z_tolerance = 30.0
        self.roll_tolerance = 5.0
        self.pitch_tolerance = 5.0
        self.yaw_tolerance = 5.0
        self.navigate_time = 2
        self.scan_wait_time = 2
        self.scans = {
            'circular': {
                'relative_setpoints_list': [
                    [0, -85, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, -22.5],
                    [0, -85, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, -22.5],
                    [0, -85, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, -22.5],
                    [0, -85, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, -22.5]
                ]
            },
            'up_and_down': {
                'relative_setpoints_list': [
                    [0, 0, 75, 0, 0, 0],
                    [0, 0, -150, 0, 0, 0],
                    [0, 0, 75, 0, 0, 0]
                ]
            },
            'something': {
                'relative_setpoints_list': [
                    [0, -50, 0, 0, 0, 0],
                    [0, 100, 0, 0, 0, 0],
                    [0, -50, 0, 0, 0, 0]
                ]
            },
            'vertical_spiral': {
                'relative_setpoints_list': [
                    [0, 0, 50, 0, 0, 0],
                    [0, -50, 0, 0, 0, 0],
                    [0, 0, -100, 0, 0, 0],
                    [0, 100, 0, 0, 0, 0],
                    [0, 0, 100, 0, 0, 0],
                    [0, -50, 0, 0, 0, 0],
                    [0, 0, -50, 0, 0, 0]
                ]
            }
        }

    def get_navigator_params(self):
        return (
            self.xy_tolerance,
            self.z_tolerance,
            self.roll_tolerance,
            self.pitch_tolerance,
            self.yaw_tolerance,
            self.navigate_time,
            self.scan_wait_time,
            self.scans
        )