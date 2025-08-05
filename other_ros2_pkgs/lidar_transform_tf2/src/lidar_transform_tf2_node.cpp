#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class LidarTransformTF2Node : public rclcpp::Node
{
public:
    LidarTransformTF2Node()
        : Node("lidar_transform_tf2_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Define LiDAR topics
        lidar_topics_ = {
            "/os_lidar_top/points",
            "/os_lidar_left/points",
            "/os_lidar_right/points",
            "/os_lidar_front/points",
            "/os_lidar_back/points"};
           
        // Initialize subscribers
        for (const auto &topic : lidar_topics_)
        {
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, 10,
                std::bind(&LidarTransformTF2Node::pointCloudCallback, this, std::placeholders::_1));
            subscribers_.push_back(sub);
        }

        // Publisher for aggregated point cloud
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_transform_tf2/points", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            // Transform the incoming cloud to the target frame
            auto transformed_cloud = tf_buffer_.transform(*msg, "yride_lidar", tf2::durationFromSec(0.1));

            // Here you can aggregate the transformed clouds
            // For simplicity, we'll publish each transformed cloud directly
            cloud_pub_->publish(transformed_cloud);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud: %s", ex.what());
        }
    }

    // Member variables
    std::vector<std::string> lidar_topics_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarTransformTF2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
