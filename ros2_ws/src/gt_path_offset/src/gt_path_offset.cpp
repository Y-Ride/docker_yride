#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp> // For PCL conversions
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h> // For converting ROS2 PointCloud2 to PCL
#include <mutex>
#include <numeric> // For std::accumulate
#include <limits>  // For std::numeric_limits

// Include the custom service header
#include "gt_path_offset/srv/save_path.hpp"

/**
 * @brief PathOffsetNode class
 *
 * This node subscribes to two nav_msgs/msg/Path topics (ground truth and robot path),
 * calculates a Z-offset from the first N points (aligned by timestamp), applies this
 * offset to the ground truth path, and publishes the new offset path. It also provides
 * a service to save the offset path as a .pcd file.
 */
class PathOffsetNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for PathOffsetNode.
     * @param options Node options.
     */
    PathOffsetNode(const rclcpp::NodeOptions &options)
        : Node("gt_path_offset_node", options),
          z_offset_(0.0),
          offset_calculated_(false)
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("gt_path_topic", "gt_path");
        this->declare_parameter<std::string>("robot_path_topic", "robot_path");
        this->declare_parameter<std::string>("offset_path_topic", "offset_path");
        this->declare_parameter<int>("offset_calculation_points", 100);

        // Get parameter values
        gt_path_topic_name_ = this->get_parameter("gt_path_topic").as_string();
        robot_path_topic_name_ = this->get_parameter("robot_path_topic").as_string();
        offset_path_topic_name_ = this->get_parameter("offset_path_topic").as_string();
        offset_calculation_points_ = this->get_parameter("offset_calculation_points").as_int();

        // Initialize subscribers
        gt_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            gt_path_topic_name_,
            10, // QoS history depth
            std::bind(&PathOffsetNode::gtPathCallback, this, std::placeholders::_1));

        robot_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            robot_path_topic_name_,
            10, // QoS history depth
            std::bind(&PathOffsetNode::robotPathCallback, this, std::placeholders::_1));

        // Initialize publisher
        offset_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            offset_path_topic_name_,
            10); // QoS history depth

        // Initialize service server
        save_path_service_ = this->create_service<gt_path_offset::srv::SavePath>(
            "save_offset_path",
            std::bind(&PathOffsetNode::savePathService, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "PathOffsetNode initialized.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to '%s' and '%s'",
                    gt_path_topic_name_.c_str(), robot_path_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", offset_path_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Offset will be calculated using the first %d points, aligned by timestamp.",
                    offset_calculation_points_);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr gt_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr robot_path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr offset_path_pub_;
    rclcpp::Service<gt_path_offset::srv::SavePath>::SharedPtr save_path_service_;

    nav_msgs::msg::Path gt_path_;
    nav_msgs::msg::Path robot_path_;
    nav_msgs::msg::Path offset_path_; // Stores the last published offset path for saving

    double z_offset_;
    bool offset_calculated_;

    std::string gt_path_topic_name_;
    std::string robot_path_topic_name_;
    std::string offset_path_topic_name_;
    int offset_calculation_points_;

    std::mutex path_mutex_; // Mutex to protect shared path data

    /**
     * @brief Callback for the ground truth path topic.
     * @param msg The received Path message.
     */
    void gtPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        gt_path_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Received GT path with %zu points.", gt_path_.poses.size());
        processPaths();
    }

    /**
     * @brief Callback for the robot path topic.
     * @param msg The received Path message.
     */
    void robotPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        robot_path_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Received Robot path with %zu points.", robot_path_.poses.size());
        processPaths();
    }

    /**
     * @brief Finds the pose in a path that is closest in time to a given timestamp.
     * @param path The path to search within.
     * @param target_stamp The timestamp to match.
     * @return The pose from the path that is closest in time, or a default-constructed PoseStamped if path is empty.
     */
    geometry_msgs::msg::PoseStamped findClosestPoseByTimestamp(const nav_msgs::msg::Path& path, const rclcpp::Time& target_stamp)
    {
        if (path.poses.empty()) {
            return geometry_msgs::msg::PoseStamped();
        }

        double min_diff_ns = std::numeric_limits<double>::max();
        geometry_msgs::msg::PoseStamped closest_pose;

        for (const auto& pose_stamped : path.poses)
        {
            rclcpp::Time pose_stamp(pose_stamped.header.stamp);
            double diff_ns = std::abs((target_stamp - pose_stamp).nanoseconds());

            if (diff_ns < min_diff_ns)
            {
                min_diff_ns = diff_ns;
                closest_pose = pose_stamped;
            }
        }
        return closest_pose;
    }

    /**
     * @brief Processes the ground truth and robot paths to calculate and apply the Z-offset.
     *
     * This function is called whenever either path is updated. It calculates the offset
     * if not already done, applies it to the ground truth path, and publishes the result.
     * It aligns points by timestamp for the offset calculation.
     */
    void processPaths()
    {
        // Ensure both paths have at least one point to proceed
        if (gt_path_.poses.empty() || robot_path_.poses.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for both GT and Robot paths to be non-empty.");
            return;
        }

        // Calculate offset if not already calculated
        if (!offset_calculated_)
        {
            double sum_z_diff = 0.0;
            int valid_points_for_offset = 0;

            // Iterate through the first N points of the GT path for offset calculation
            for (size_t i = 0; i < gt_path_.poses.size() && i < (size_t)offset_calculation_points_; ++i)
            {
                const auto& gt_pose = gt_path_.poses[i];
                rclcpp::Time gt_stamp(gt_pose.header.stamp);

                // Find the closest robot pose by timestamp
                geometry_msgs::msg::PoseStamped closest_robot_pose = findClosestPoseByTimestamp(robot_path_, gt_stamp);

                // Check if a valid closest pose was found (i.e., not default-constructed)
                if (closest_robot_pose.header.stamp.sec != 0 || closest_robot_pose.header.stamp.nanosec != 0)
                {
                    double gt_z = gt_pose.pose.position.z;
                    double robot_z = closest_robot_pose.pose.position.z;
                    sum_z_diff += (gt_z - robot_z);
                    valid_points_for_offset++;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Could not find a corresponding robot pose for GT pose at index %zu (timestamp: %f).",
                                i, gt_stamp.seconds());
                }
            }

            if (valid_points_for_offset > 0)
            {
                z_offset_ = sum_z_diff / valid_points_for_offset;
                offset_calculated_ = true;
                RCLCPP_INFO(this->get_logger(), "Calculated Z-offset: %f based on %d timestamp-aligned points.", z_offset_, valid_points_for_offset);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No valid corresponding points found to calculate Z-offset.");
                return; // Cannot proceed without offset
            }
        }

        // Apply offset and publish the new path
        nav_msgs::msg::Path offset_path_msg;
        offset_path_msg.header = gt_path_.header; // Use GT path's header

        for (const auto &pose_stamped : gt_path_.poses)
        {
            geometry_msgs::msg::PoseStamped new_pose_stamped = pose_stamped;
            // Apply constant offset only to the Z-direction
            new_pose_stamped.pose.position.z -= z_offset_;
            offset_path_msg.poses.push_back(new_pose_stamped);
        }

        offset_path_pub_->publish(offset_path_msg);
        offset_path_ = offset_path_msg; // Store for service call
        RCLCPP_DEBUG(this->get_logger(), "Published offset path with %zu points.", offset_path_msg.poses.size());
    }

    /**
     * @brief Service callback to save the current offset_path as a .pcd file.
     * @param request The service request containing the filename.
     * @param response The service response indicating success or failure.
     */
    void savePathService(const std::shared_ptr<gt_path_offset::srv::SavePath::Request> request,
                         std::shared_ptr<gt_path_offset::srv::SavePath::Response> response)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);

        if (offset_path_.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cannot save empty offset path.");
            response->success = false;
            return;
        }

        // Create a PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = offset_path_.poses.size();
        cloud.height = 1; // Unordered cloud
        cloud.is_dense = true; // No NaNs

        for (const auto &pose_stamped : offset_path_.poses)
        {
            cloud.points.push_back(pcl::PointXYZ(
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z));
        }

        // Save the PCD file
        try
        {
            pcl::io::savePCDFileASCII(request->filename, cloud);
            RCLCPP_INFO(this->get_logger(), "Successfully saved offset path to '%s'.", request->filename.c_str());
            response->success = true;
        }
        catch (const pcl::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file '%s': %s", request->filename.c_str(), e.what());
            response->success = false;
        }
    }
};

// Main function to spin the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<PathOffsetNode>(rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
