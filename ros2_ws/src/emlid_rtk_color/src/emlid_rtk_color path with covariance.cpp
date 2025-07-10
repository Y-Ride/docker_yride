#include <cstdio>
#include <vector>
#include <functional>
#include <cmath> // For std::sqrt

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


class EmlidRtkColor : public rclcpp::Node
{
public:
  EmlidRtkColor(const std::string & name, const std::string & ns, const rclcpp::NodeOptions & options) : Node(name, ns, options)
  {
    this->declare_parameter("navsatfix_topic", "/emlid/NavSatFix");
    this->declare_parameter("path_topic", "gps_path");
    this->declare_parameter("covariance_markers_topic", "gps_covariance_markers");
    this->declare_parameter("origin_lat", 40.246266148);
    this->declare_parameter("origin_lon", -111.648571401);
    this->declare_parameter("origin_alt", 1403.8386);
    this->declare_parameter("marker_scale_covariance", 1.0);

    std::string navsatfix_topic, path_topic, covariance_markers_topic;
    double origin_lat, origin_lon, origin_alt;
    this->get_parameter("navsatfix_topic", navsatfix_topic);
    this->get_parameter("path_topic", path_topic);
    this->get_parameter("covariance_markers_topic", covariance_markers_topic);
    this->get_parameter("origin_lat", origin_lat);
    this->get_parameter("origin_lon", origin_lon);
    this->get_parameter("origin_alt", origin_alt);
    this->get_parameter("marker_scale_covariance", marker_scale_covariance_);

    local_cartesian_.Reset(origin_lat, origin_lon, origin_alt);

    sub_navsatfix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      navsatfix_topic,
      10,
      std::bind(&EmlidRtkColor::NavSatFixCallback, this, std::placeholders::_1)
    );

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      path_topic,
      10
    );

    pub_covariance_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      covariance_markers_topic,
      10
    );

    save_pcd_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_gps_data_to_pcd",
      std::bind(&EmlidRtkColor::saveGpsDataToPCDService, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Initialize path message
    path_msg_.header.frame_id = "map";
    // REMOVED: covariance_marker_array_.header.frame_id = "map"; // This line caused the error
  }

  ~EmlidRtkColor()
  {
    savePathToPCD("/gps_path_points.pcd");
    saveCovarianceMarkersToPCD("/gps_covariance_markers.pcd");
  }

private:

  void NavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "RTK GPS status: %d", msg->status.status);

    double x, y, z;
    local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    // 1. Update and Publish Path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = z;
    pose_stamped.pose.orientation.w = 1.0;

    path_msg_.poses.push_back(pose_stamped);
    path_msg_.header.stamp = msg->header.stamp;
    pub_path_->publish(path_msg_);

    // 2. Create and Store Covariance Marker
    visualization_msgs::msg::Marker covariance_marker;
    covariance_marker.header = msg->header; // Each marker has its own header
    covariance_marker.header.frame_id = "map";
    covariance_marker.ns = "gps_covariance";
    covariance_marker.id = marker_id_counter_++;
    covariance_marker.type = visualization_msgs::msg::Marker::CUBE;
    covariance_marker.action = visualization_msgs::msg::Marker::ADD;
    covariance_marker.pose.position = pose_stamped.pose.position;
    covariance_marker.pose.orientation.w = 1.0;

    double std_dev_x = std::sqrt(msg->position_covariance[0]);
    double std_dev_y = std::sqrt(msg->position_covariance[4]);
    double std_dev_z = std::sqrt(msg->position_covariance[8]);

    covariance_marker.scale.x = marker_scale_covariance_ * 2.0 * std_dev_x;
    covariance_marker.scale.y = marker_scale_covariance_ * 2.0 * std_dev_y;
    covariance_marker.scale.z = marker_scale_covariance_ * 2.0 * std_dev_z;

    covariance_marker.color = rtkStatusToColor(msg->status.status);
    covariance_marker.lifetime = rclcpp::Duration::from_seconds(0);

    covariance_marker_array_.markers.push_back(covariance_marker);
    // REMOVED: covariance_marker_array_.header.stamp = msg->header.stamp; // This line also caused the error
    pub_covariance_markers_->publish(covariance_marker_array_);
  }

  std_msgs::msg::ColorRGBA rtkStatusToColor(int status) {
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    switch (status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        break;
      case 5: // RTK Float as per defined in emlid_gps.hpp
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      case 4: // RTK FIX as per defined in emlid_gps.hpp
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Invalid RTK Status: %d", status);
        color.r = 0.5; color.g = 0.5; color.b = 0.5;
    }

    return color;
  }

  void savePathToPCD(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& pose_stamped : path_msg_.poses) {
      pcl::PointXYZRGB pcl_point;
      pcl_point.x = pose_stamped.pose.position.x;
      pcl_point.y = pose_stamped.pose.position.y;
      pcl_point.z = pose_stamped.pose.position.z;

      uint8_t r = 255; uint8_t g = 255; uint8_t b = 255;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                      static_cast<uint32_t>(g) << 8 |
                      static_cast<uint32_t>(b));
      pcl_point.rgb = *reinterpret_cast<float*>(&rgb);

      cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved %zu path points to %s", cloud->size(), filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file to %s", filename.c_str());
    }
  }

  void saveCovarianceMarkersToPCD(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& marker : covariance_marker_array_.markers) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = marker.pose.position.x;
        pcl_point.y = marker.pose.position.y;
        pcl_point.z = marker.pose.position.z;

        uint8_t r = static_cast<uint8_t>(marker.color.r * 255);
        uint8_t g = static_cast<uint8_t>(marker.color.g * 255);
        uint8_t b = static_cast<uint8_t>(marker.color.b * 255);
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 |
                        static_cast<uint32_t>(b));
        pcl_point.rgb = *reinterpret_cast<float*>(&rgb);

        cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved %zu colored covariance marker points to %s", cloud->size(), filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file to %s", filename.c_str());
    }
  }


  void saveGpsDataToPCDService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void) request;

    std::string path_filename = "/gps_path_points_service.pcd";
    std::string covariance_filename = "/gps_covariance_markers_service.pcd";
    try {
      savePathToPCD(path_filename);
      saveCovarianceMarkersToPCD(covariance_filename);
      response->success = true;
      response->message = "Saved path points to " + path_filename + " and covariance markers to " + covariance_filename;
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to save PCD: " + std::string(e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_navsatfix_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_covariance_markers_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_service_;

  GeographicLib::LocalCartesian local_cartesian_;
  nav_msgs::msg::Path path_msg_;
  visualization_msgs::msg::MarkerArray covariance_marker_array_;
  int marker_id_counter_ = 0;
  float marker_scale_covariance_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<EmlidRtkColor>("emlid_rtk_color", "emlid_color", options));
  rclcpp::shutdown();
  return 0;
}