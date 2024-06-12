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

        self.yaw = 0.0
        self.encoder_speed = 0.0

    def imu_callback(self, msg):
        # Extract yaw from IMU data (converting quaternion to euler angles)
        self.yaw = self.quaternion_to_euler_yaw(msg.orientation)

    def quaternion_to_euler_yaw(self, orientation):
        # Convert quaternion to euler yaw angle
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def encoder_callback(self, msg):
        self.encoder_speed = msg.data
        self.calculate_speed()

    def calculate_speed(self):
        # Adjust encoder speed using yaw angle
        adjusted_speed = self.encoder_speed * np.cos(self.yaw)

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

