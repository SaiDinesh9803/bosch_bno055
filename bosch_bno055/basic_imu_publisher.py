import rclpy
from rclpy.node import Node
from bosch_bno055_interfaces.msg import Num
import board
import busio
import adafruit_bno055

class BNO055Publisher(Node):

    def __init__(self):
        self.i2c = busio.I2C(board.SCL , board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        super().__init__('bno055_publisher')
        self.publisher_imu_raw = self.create_publisher(Num, '/bno055/linear_acceleration', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()
        
        msg.acceleration.x = self.sensor.linear_acceleration[0]
        msg.acceleration.y = self.sensor.linear_acceleration[1]
        msg.acceleration.z = self.sensor.linear_acceleration[2]
        self.publisher_imu_raw.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    bno055_publisher = BNO055Publisher()

    rclpy.spin(bno055_publisher)

    bno055_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()