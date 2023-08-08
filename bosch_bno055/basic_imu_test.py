import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import board
import busio
import adafruit_bno055

class BNO055Publisher(Node):

    def __init__(self):
        self.i2c = busio.I2C(board.SCL , board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.last_val = 0xFFFF

        super().__init__('bno055_publisher')
        self.publisher_imu_raw = self.create_publisher(Imu, '/imu', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
    def temperature(self):
        result = self.sensor.temperature
        if abs(result - self.last_val) == 128:
            result = self.sensor.temperature
        if abs(result - self.last_val) == 128:
            return 0b00111111 & result
        self.last_val = result
        return result

    def timer_callback(self):
        msg = Imu()
        
        quat = Quaternion()
        quat.x = self.sensor.quaternion[0]
        quat.y = self.sensor.quaternion[1]
        quat.z = self.sensor.quaternion[2]
        quat.w = self.sensor.quaternion[3]
        
        msg.orientation = quat
        
        ang_vel = Vector3()
        ang_vel.x = self.sensor.gyro[0]
        ang_vel.y = self.sensor.gyro[1]
        ang_vel.z = self.sensor.gyro[2]
        
        msg.angular_velocity = ang_vel
        
        lin_acc = Vector3()
        lin_acc.x = self.sensor.linear_acceleration[0]
        lin_acc.y = self.sensor.linear_acceleration[1]
        lin_acc.z = self.sensor.linear_acceleration[2]
        
        msg.linear_acceleration = lin_acc
        
        
        
        self.publisher_imu_raw.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    bno055_publisher = BNO055Publisher()

    rclpy.spin(bno055_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bno055_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()










# while True:
#     print("Temperature: {} degrees C".format(sensor.temperature))
#     """
#     print(
#         "Temperature: {} degrees C".format(temperature())
#     )  # Uncomment if using a Raspberry Pi
#     """
#     print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
#     print("Magnetometer (microteslas): {}".format(sensor.magnetic))
#     print("Gyroscope (rad/sec): {}".format(sensor.gyro))
#     print("Euler angle: {}".format(sensor.euler))
#     print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
#     print("Gravity (m/s^2): {}".format(sensor.gravity))
#     print()

#     time.sleep(1)
