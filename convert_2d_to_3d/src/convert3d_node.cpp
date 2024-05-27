#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

class Convert3D : public rclcpp::Node
{
public:
    Convert3D() : Node("convert3d_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&Convert3D::tfCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Convert3D::scanCallback, this, std::placeholders::_1));
        encoder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/encoder", 10, std::bind(&Convert3D::encoderCallback, this, std::placeholders::_1));
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_3d", 10);
    }

private:
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto &transform : msg->transforms)
        {
            std::string child_frame_id = transform.child_frame_id;
            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;
            current_z_ = 1.0; // 라이다의 높이

            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // 방향을 롤, 피치, 요로 변환
            yaw_rad_car_ = yaw;
        }
    }

    void encoderCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        pitch_rad_encoder_ = msg->data; // !!!만약 msg가 rad가 아닌 deg로 들어온다면 변환 필요!!!
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        laser_frame_ = scan->header.frame_id;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        //
        std::size_t count = 0;
        float min_rad = scan->angle_min;
        float max_rad = scan->angle_max;
        for (float i = min_rad; i <= max_rad && count < scan->ranges.size(); i += scan->angle_increment, ++count)
        {
            pcl::PointXYZ point;
            float yaw = i + yaw_rad_car_;
            float temp_x = scan->ranges[count] * std::cos(yaw); // 라이다 기준 x,y 좌표
            float temp_y = scan->ranges[count] * std::sin(yaw);

            point.x = temp_x * std::cos(pitch_rad_encoder_) + current_x_;
            point.y = temp_y + current_y_;
            point.z = scan->ranges[count] * std::cos(yaw) * std::sin(pitch_rad_encoder_) + current_z_;
            cloud.push_back(point);
        }
        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg(cloud, cloud_out);
        cloud_out.header = scan->header;
        cloud_out.header.frame_id = "3d_map";
        cloud_pub_->publish(cloud_out);
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double current_x_;
    double current_y_;
    double current_z_;
    double yaw_rad_car_;
    double pitch_rad_encoder_;
    std::string laser_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Convert3D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
