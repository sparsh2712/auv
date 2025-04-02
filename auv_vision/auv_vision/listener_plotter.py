import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from auv_msgs.msg import VisionSetpoints

class PeePeePooPoo(Node):
    def __init__(self):
        super().__init__('listener_plotter')
        self.subscription = self.create_subscription(VisionSetpoints, '/vision/detection', self.listener_callback, 10)
        self.data = []

    def listener_callback(self, msg):
        for object in msg.objects:
            self.data.append(object.position.z)
            self.get_logger().info(f'Received: {object.position.z}')
            plt.plot(range(len(self.data)), self.data)
            plt.savefig('plot.png')


def main(args=None):
    rclpy.init(args=args)
    listener_plotter = PeePeePooPoo()
    try:
        rclpy.spin(listener_plotter)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
