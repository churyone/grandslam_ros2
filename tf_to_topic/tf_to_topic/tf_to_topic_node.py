import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class TFToOdometryNode(Node):
    def __init__(self):
        super().__init__('tf_to_topic_node')
        
        # Create a buffer and transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a publisher for the odometry data
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create a timer to call the callback function periodically
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('TF to Topic node has been started')

    def timer_callback(self):
        try:
            # 트랜스폼 조회 시점의 로그 추가
            now = rclpy.time.Time()
            self.get_logger().info(f'Looking up transform at time: {now}')
            
            # odom에서 base_footprint로의 트랜스폼을 조회
            transform: TransformStamped = self.tf_buffer.lookup_transform('odom', 'base_footprint', now)
            
            # 트랜스폼 수신 로그 추가
            self.get_logger().info(f'Transform received: {transform}')
            
            # Create an Odometry message and populate it with the transform data
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'
            
            # 트랜스폼 데이터를 사용하여 오도메트리 메시지를 채움
            odom_msg.pose.pose.position.x = transform.transform.translation.x
            odom_msg.pose.pose.position.y = transform.transform.translation.y
            odom_msg.pose.pose.position.z = transform.transform.translation.z
            odom_msg.pose.pose.orientation = transform.transform.rotation
            
            # 오도메트리 메시지 퍼블리시
            self.odom_pub.publish(odom_msg)
            self.get_logger().info('Published Odometry message')
            
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TFToOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
