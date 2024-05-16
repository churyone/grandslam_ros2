import rclpy
from rclpy.node import Node
#TransformListener는 TF데이터를 수신, Buffer는 TF데이터를 저장하고 관리
from tf2_ros import TransformListener, Buffer
#Odometry타입 메시지 사용
from nav_msgs.msg import Odometry
#TF트랜스폼 데이터 나타내기 위해 사용
from geometry_msgs.msg import TransformStamped

#Node 클래스를 상속받아 TFToOdometryNode 정의
class TFToOdometryNode(Node):
    #클래스 초기화 메소드 정의, 부모 클래스인 노드를 초기화 후 노드의 이름을 tf_to_odometry_node로 설정, 이때 self는 인스턴스 자신을 참조
    def __init__(self):
        super().__init__('tf_to_odometry_node')
        
        # Create a buffer and transform listener
        #인스턴스 자신에 tf_buffer라는 객체 생성, Buffer()를 생성자로 함
        self.tf_buffer = Buffer()
        #인스턴스 자신에 tf_listener라는 객체 생성, TransformListener()를 생성자로 함
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a publisher for the odometry data
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create a timer to call the callback function periodically
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('TF to Odometry node has been started')

    def timer_callback(self):
        try:
            # Lookup the transform from map to base_link
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            
            # Create an Odometry message and populate it with the transform data
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            odom_msg.pose.pose.position.x = transform.transform.translation.x
            odom_msg.pose.pose.position.y = transform.transform.translation.y
            odom_msg.pose.pose.position.z = transform.transform.translation.z
            odom_msg.pose.pose.orientation = transform.transform.rotation
            
            # Publish the Odometry message
            self.odom_pub.publish(odom_msg)
            
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
