import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from bosch_bno055_interfaces.msg import Num

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,
            '/bno055/linear_acceleration',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.plotx = []
        self.ploty = []
        self.i = 0
        #plt.show()

    def listener_callback(self, msg):
        self.ploty.append(msg.acceleration.x)
        #plt.plot(self.plotx , self.ploty)
        plt.plot(self.i , self.ploty[self.i])
        self.i+=1


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()