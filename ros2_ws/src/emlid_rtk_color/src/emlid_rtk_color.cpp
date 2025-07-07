#include <cstdio>
// #include "emlid_gps/emlid_gps.hpp"
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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
    this->declare_parameter("colored_markers_topic", "colored_markers");
    this->declare_parameter("origin_lat", 40.246266148);
    this->declare_parameter("origin_lon", -111.648571401);
    this->declare_parameter("origin_alt", 1403.8386);
    this->declare_parameter("marker_scale", 1.0);

    std::string navsatfix_topic, colored_markers_topic;
    double origin_lat, origin_lon, origin_alt;
    this->get_parameter("navsatfix_topic", navsatfix_topic);
    this->get_parameter("colored_markers_topic", colored_markers_topic);
    this->get_parameter("origin_lat", origin_lat);
    this->get_parameter("origin_lon", origin_lon);
    this->get_parameter("origin_alt", origin_alt);
    this->get_parameter("marker_scale", marker_scale);

    local_cartesian_.Reset(origin_lat, origin_lon, origin_alt);

    sub_navsatfix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      navsatfix_topic,
      10,
      std::bind(&EmlidRtkColor::NavSatFixCallback, this, std::placeholders::_1)
    );

    pub_color_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      colored_markers_topic,
      10
    );

    save_pcd_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_markers_to_pcd",
      std::bind(&EmlidRtkColor::saveMarkersToPCDService, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  ~EmlidRtkColor()
  {
    saveMarkersToPCD("/gps_path_markers.pcd");
  }

private:

  void NavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    
    if (pub_color_marker_->get_subscription_count() == 0) {
      marker_array_.markers.clear();
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "RTK GPS status: %d", msg->status.status);

    double x, y, z;
    local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    path_points_.push_back(point);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = msg->header.stamp;
    marker.ns = "gps_points";
    marker.id = marker_id_counter_++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position = point;
    // marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.points = path_points_;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale;
    marker.scale.y = marker_scale;
    marker.scale.z = marker_scale;
    marker.color = rtkStatusToColor(msg->status.status);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker_array_.markers.push_back(marker);
    pub_color_marker_->publish(marker_array_);

  }

  std_msgs::msg::ColorRGBA rtkStatusToColor(int status) {
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    switch (status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
        // White
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
        // Red
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        break;
      case 5: // RTK Float as per defined in emlid_gps.hpp
        // Yellow
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      case 4: // RTK FIX as per defined in emlid_gps.hpp
        // Green
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Invalid RTK Status");
    }

    return color;
  }

  void saveMarkersToPCD(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  
    for (const auto& marker : marker_array_.markers) {
      for (const auto& pt : marker.points) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = pt.x;
        pcl_point.y = pt.y;
        pcl_point.z = pt.z;
  
        // Use marker color for each point
        uint8_t r = static_cast<uint8_t>(marker.color.r * 255);
        uint8_t g = static_cast<uint8_t>(marker.color.g * 255);
        uint8_t b = static_cast<uint8_t>(marker.color.b * 255);
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 |
                        static_cast<uint32_t>(b));
        pcl_point.rgb = *reinterpret_cast<float*>(&rgb);
  
        cloud->points.push_back(pcl_point);
      }
    }
  
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
  
    if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved %zu colored marker points to %s", cloud->size(), filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file to %s", filename.c_str());
    }
  }

  void saveMarkersToPCDService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void) request;

    std::string filename = "/gps_markers.pcd";
    try {
      saveMarkersToPCD(filename);
      response->success = true;
      response->message = "Saved markers to " + filename;
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to save PCD: " + std::string(e.what());
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_navsatfix_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_color_marker_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_service_;

  GeographicLib::LocalCartesian local_cartesian_;
  bool origin_set_ = false;
  double origin_lat_, origin_lon_, origin_alt_;
  std::vector<geometry_msgs::msg::Point> path_points_;
  visualization_msgs::msg::MarkerArray marker_array_;
  int marker_id_counter_ = 0;
  float marker_scale;


};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<EmlidRtkColor>("emlid_rtk_color", "emlid_color", options));
  rclcpp::shutdown();
  return 0;
}






