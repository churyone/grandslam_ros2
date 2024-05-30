import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np

class SpeedEstimator(Node):
    def __init__(self):
        super().__init__('speed_estimator')
        self.subscription_imu = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.subscription_encoder = self.create_subscription(Float32, 'speed_topic', self.encoder_callback, 10)
        self.publisher_speed = self.create_publisher(Float32, 'vehicle/speed', 10)

        self.roll = 0.0
        self.encoder_speed = 0.0

    def imu_callback(self, msg):
        # Extract roll from IMU data
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        self.roll = np.arctan2(accel_y, accel_z)

    def encoder_callback(self, msg):
        self.encoder_speed = msg.data
        self.calculate_speed()

    def calculate_speed(self):
        # Adjust encoder speed using roll angle
        adjusted_speed = self.encoder_speed * np.cos(self.roll)

        # Publish the adjusted speed
        speed_msg = Float32()
        speed_msg.data = adjusted_speed
        self.publisher_speed.publish(speed_msg)

        self.get_logger().info(f'Adjusted Speed: {adjusted_speed:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    speed_estimator = SpeedEstimator()
    rclpy.spin(speed_estimator)
    speed_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
