#include <cstdio>
#include <vector>
#include <functional>
#include <cmath>
#include <deque> // For std::deque
#include <limits> // For std::numeric_limits
#include <fstream> // For std::ofstream
#include <array>   // For std::array

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp" // For subscribing to mapping path
#include <GeographicLib/LocalCartesian.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// For Eigen operations (eigenvalue decomposition for covariance visualization)
#include <Eigen/Dense>


class EmlidRtkColor : public rclcpp::Node
{
public:
  EmlidRtkColor(const std::string & name, const std::string & ns, const rclcpp::NodeOptions & options) : Node(name, ns, options)
  {
    // Declare parameters
    this->declare_parameter("navsatfix_topic", "/emlid/NavSatFix");
    this->declare_parameter("colored_path_markers_topic", "colored_gps_path");
    this->declare_parameter("covariance_markers_topic", "gps_covariance_markers");
    this->declare_parameter("origin_lat", 40.246266148);
    this->declare_parameter("origin_lon", -111.648571401);
    this->declare_parameter("origin_alt", 1403.8386);
    this->declare_parameter("marker_scale_covariance", 1.0);
    this->declare_parameter("line_strip_scale", 0.1);
    this->declare_parameter("mapping_path_topic", "/liorf/mapping/path");
    this->declare_parameter("fixed_alignment_buffer_size", 100); // Parameter for buffer size N
    this->declare_parameter("max_initial_xy_jump", 3.0); // Parameter for jump detection threshold

    // Get parameter values
    std::string navsatfix_topic, colored_path_markers_topic, covariance_markers_topic, mapping_path_topic;
    double origin_lat, origin_lon, origin_alt;
    this->get_parameter("navsatfix_topic", navsatfix_topic);
    this->get_parameter("colored_path_markers_topic", colored_path_markers_topic);
    this->get_parameter("covariance_markers_topic", covariance_markers_topic);
    this->get_parameter("origin_lat", origin_lat);
    this->get_parameter("origin_lon", origin_lon);
    this->get_parameter("origin_alt", origin_alt);
    this->get_parameter("marker_scale_covariance", marker_scale_covariance_);
    this->get_parameter("line_strip_scale", line_strip_scale_);
    this->get_parameter("mapping_path_topic", mapping_path_topic);
    this->get_parameter("fixed_alignment_buffer_size", fixed_alignment_buffer_size_);
    this->get_parameter("max_initial_xy_jump", max_initial_xy_jump_);

    // Initialize GeographicLib LocalCartesian for coordinate transformations
    local_cartesian_.Reset(origin_lat, origin_lon, origin_alt);

    // Create subscriptions
    sub_navsatfix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      navsatfix_topic,
      10,
      std::bind(&EmlidRtkColor::NavSatFixCallback, this, std::placeholders::_1)
    );

    sub_mapping_path_ = this->create_subscription<nav_msgs::msg::Path>(
      mapping_path_topic,
      10,
      std::bind(&EmlidRtkColor::MappingPathCallback, this, std::placeholders::_1)
    );

    // Create publishers
    pub_colored_path_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      colored_path_markers_topic,
      10
    );

    pub_covariance_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      covariance_markers_topic,
      10
    );

    // Create service for saving PCD data
    save_pcd_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_gps_data_to_pcd",
      std::bind(&EmlidRtkColor::saveGpsDataToPCDService, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Initialize state variables for path coloring and Z-alignment
    last_fix_status_ = -1; // Initialize to an invalid status to force creation of first line segment
    current_line_marker_id_ = -1; // No current line marker initially
    initial_offset_calculated_ = false; // Offset not calculated at start
    fixed_z_offset_ = 0.0; // Initial fixed Z offset is zero
    first_initial_mapping_point_ = true; // Flag for the very first mapping point in the initial collection phase
  }

  // Destructor to save data on node shutdown
  ~EmlidRtkColor()
  {
    savePathMarkersToPCD("/gps_path_markers_shutdown.pcd");
    saveCovarianceAsLinesToPLY("/gps_covariance_lines_shutdown.ply"); // Save covariance lines on shutdown
  }

private:
  /**
   * @brief Calculates the fixed Z-axis offset based on the average Z of initial RTK and mapping points.
   * This method is called once both initial buffers are filled.
   */
  void calculateFixedZOffset()
  {
    double sum_rtk_z = 0.0;
    for (const auto& pt : initial_rtk_points_buffer_) {
      sum_rtk_z += pt.z;
    }
    double avg_rtk_z = sum_rtk_z / initial_rtk_points_buffer_.size();

    double sum_mapping_z = 0.0;
    for (const auto& pt : initial_mapping_points_buffer_) {
      sum_mapping_z += pt.z;
    }
    double avg_mapping_z = sum_mapping_z / initial_mapping_points_buffer_.size();

    // The offset to apply to RTK_Z to align it with Mapping_Z
    // If RTK is higher than mapping, this value will be positive.
    // E.g., if RTK_avg = 10m, Mapping_avg = 5m, offset = 5m. Then RTK_Z_aligned = RTK_Z - 5m.
    fixed_z_offset_ = avg_rtk_z - avg_mapping_z;

    RCLCPP_INFO(this->get_logger(),
                "Calculated initial average RTK Z: %f, average Mapping Z: %f",
                avg_rtk_z, avg_mapping_z);
    RCLCPP_INFO(this->get_logger(),
                "Fixed Z-offset calculated: %f. This offset will be applied to all subsequent ground truth points.",
                fixed_z_offset_);

    // Retroactively update all previously published markers and stored data
    rclcpp::Time current_time = this->now();

    // 1. Update colored_path_marker_array_
    for (auto& marker : colored_path_marker_array_.markers) {
        for (auto& point : marker.points) {
            point.z -= fixed_z_offset_;
        }
        marker.header.stamp = current_time; // Update timestamp for RViz to re-process
    }
    pub_colored_path_->publish(colored_path_marker_array_); // Re-publish the updated array

    // 2. Update covariance_marker_array_
    for (auto& marker : covariance_marker_array_.markers) {
        marker.pose.position.z -= fixed_z_offset_;
        marker.header.stamp = current_time; // Update timestamp for RViz to re-process
    }
    pub_covariance_markers_->publish(covariance_marker_array_); // Re-publish the updated array

    // 3. Update covariance_data_for_ply_ (for PLY export)
    for (auto& cov_data : covariance_data_for_ply_) {
        cov_data.position.z -= fixed_z_offset_;
    }

    // Clear buffers as they are no longer needed after calculation
    initial_rtk_points_buffer_.clear();
    initial_mapping_points_buffer_.clear();
  }

  /**
   * @brief Callback for incoming NavSatFix messages.
   * Processes the RTK GPS data, calculates local Cartesian coordinates,
   * performs Z-axis alignment (after initial offset calculation), and
   * publishes colored path segments and covariance markers.
   * @param msg Shared pointer to the NavSatFix message.
   */
  void NavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "RTK GPS status: %d", msg->status.status);

    // Convert latitude, longitude, altitude to local Cartesian (x, y, z)
    double x, y, z_rtk;
    local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z_rtk);

    double aligned_z = z_rtk; // Default to original Z if offset not calculated yet

    // Phase 1: Collect initial RTK points if offset hasn't been calculated
    if (!initial_offset_calculated_) {
      geometry_msgs::msg::Point current_rtk_pt_for_buffer; // Use a temporary variable
      current_rtk_pt_for_buffer.x = x;
      current_rtk_pt_for_buffer.y = y;
      current_rtk_pt_for_buffer.z = z_rtk;

      // Add point to buffer if space available
      if (initial_rtk_points_buffer_.size() < fixed_alignment_buffer_size_) {
        initial_rtk_points_buffer_.push_back(current_rtk_pt_for_buffer);
        RCLCPP_DEBUG(this->get_logger(), "Collecting initial RTK points: %zu/%zu", initial_rtk_points_buffer_.size(), fixed_alignment_buffer_size_);
      }

      // Check if both initial buffers are full and calculate offset if ready
      if (initial_rtk_points_buffer_.size() == fixed_alignment_buffer_size_ &&
          initial_mapping_points_buffer_.size() == fixed_alignment_buffer_size_ &&
          !initial_offset_calculated_)
      {
        calculateFixedZOffset(); // Calculate the offset once
        initial_offset_calculated_ = true; // Set flag
      }
    }

    // Phase 2: Apply fixed offset if it has been calculated
    if (initial_offset_calculated_) {
      aligned_z = z_rtk - fixed_z_offset_; // Apply the fixed offset
    }

    geometry_msgs::msg::Point current_point;
    current_point.x = x;
    current_point.y = y;
    current_point.z = aligned_z; // Use the aligned Z for visualization and saving

    // 1. Update and Publish Colored Path Markers
    // Check if the GPS fix status has changed to start a new line segment
    if (msg->status.status != last_fix_status_) {
        // Status changed, create a new LINE_STRIP marker
        visualization_msgs::msg::Marker new_line_marker;
        new_line_marker.header = msg->header;
        new_line_marker.header.frame_id = "map"; // Ensure consistent frame_id
        new_line_marker.ns = "colored_gps_path";
        new_line_marker.id = line_marker_id_counter_++; // Assign a new unique ID for this segment
        new_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        new_line_marker.action = visualization_msgs::msg::Marker::ADD;
        new_line_marker.pose.orientation.w = 1.0; // Identity orientation
        new_line_marker.scale.x = line_strip_scale_; // Line thickness
        new_line_marker.color = rtkStatusToColor(msg->status.status); // Set color based on current status
        new_line_marker.lifetime = rclcpp::Duration::from_seconds(0); // Persist indefinitely

        // Add the current point to the new line segment
        new_line_marker.points.push_back(current_point);
        colored_path_marker_array_.markers.push_back(new_line_marker);

        // Store the ID of the newly created marker so we can append subsequent points to it
        current_line_marker_id_ = new_line_marker.id;
        last_fix_status_ = msg->status.status; // Update last known status
    } else {
        // Status is the same, append current point to the existing LINE_STRIP marker
        // Find the marker with current_line_marker_id_ and append the point
        for (auto& marker : colored_path_marker_array_.markers) {
            if (marker.id == current_line_marker_id_ && marker.ns == "colored_gps_path") {
                marker.points.push_back(current_point);
                marker.header.stamp = msg->header.stamp; // Update timestamp for current marker
                break;
            }
        }
    }
    pub_colored_path_->publish(colored_path_marker_array_); // Publish the entire array of path segments


    // 2. Create and Store Covariance Marker (CUBE type for RViz visualization)
    visualization_msgs::msg::Marker covariance_marker;
    covariance_marker.header = msg->header;
    covariance_marker.header.frame_id = "map"; // Ensure consistent frame_id
    covariance_marker.ns = "gps_covariance";
    covariance_marker.id = marker_id_counter_++; // Unique ID for each covariance marker
    covariance_marker.type = visualization_msgs::msg::Marker::CUBE;
    covariance_marker.action = visualization_msgs::msg::Marker::ADD;
    covariance_marker.pose.position = current_point; // Position at the aligned RTK point
    covariance_marker.pose.orientation.w = 1.0; // Identity orientation

    // Extract standard deviations from covariance matrix (indices 0, 4, 8 for xx, yy, zz variance)
    double std_dev_x = std::sqrt(msg->position_covariance[0]);
    double std_dev_y = std::sqrt(msg->position_covariance[4]);
    double std_dev_z = std::sqrt(msg->position_covariance[8]);

    // Scale the cube by 2 * standard deviation and apply the custom scale parameter
    covariance_marker.scale.x = marker_scale_covariance_ * 2.0 * std_dev_x;
    covariance_marker.scale.y = marker_scale_covariance_ * 2.0 * std_dev_y;
    covariance_marker.scale.z = marker_scale_covariance_ * 2.0 * std_dev_z;

    covariance_marker.color = rtkStatusToColor(msg->status.status); // Color based on RTK status
    covariance_marker.lifetime = rclcpp::Duration::from_seconds(0); // Persist indefinitely

    covariance_marker_array_.markers.push_back(covariance_marker);
    pub_covariance_markers_->publish(covariance_marker_array_); // Publish the entire array of covariance markers

    // Store covariance data for PLY export (for Open3D visualization)
    CovarianceData cov_data;
    cov_data.position = current_point;
    // Copy the 9 elements of the covariance matrix
    std::copy(msg->position_covariance.begin(), msg->position_covariance.end(), cov_data.covariance_matrix.begin());
    cov_data.color = rtkStatusToColor(msg->status.status);
    covariance_data_for_ply_.push_back(cov_data);
  }

  /**
   * @brief Callback for incoming mapping path messages.
   * Stores the latest points from the mapping path into a buffer, but only
   * until the initial alignment offset has been calculated. Includes jump detection.
   * @param msg Shared pointer to the Path message.
   */
  void MappingPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // Only collect initial mapping points if the fixed offset hasn't been calculated yet
    if (!initial_offset_calculated_ && initial_mapping_points_buffer_.size() < fixed_alignment_buffer_size_) {
      for (const auto& pose_stamped : msg->poses) {
        geometry_msgs::msg::Point current_mapping_pt = pose_stamped.pose.position;

        // Check for large jumps in initial mapping points
        // Only perform check if it's not the very first point and we have at least one point to compare against
        if (!first_initial_mapping_point_ && initial_mapping_points_buffer_.size() > 0) {
          double dx = current_mapping_pt.x - last_initial_mapping_point_.x;
          double dy = current_mapping_pt.y - last_initial_mapping_point_.y;

          if (dx > max_initial_xy_jump_ || dy > max_initial_xy_jump_) {
            RCLCPP_WARN(this->get_logger(),
                        "Large jump detected in initial Mapping Path position (dx (%f m) or dy (%f m) > %f m). Resetting initial buffers.",
                        dx, dy, max_initial_xy_jump_);
            initial_mapping_points_buffer_.clear();
            initial_rtk_points_buffer_.clear(); // Reset RTK buffer as well for re-sync
            last_initial_mapping_point_ = current_mapping_pt; // Store this point for next comparison
            first_initial_mapping_point_ = true; // Reset flag to treat next valid point as the new "first"
            return; // Exit this callback to discard the current jumping message and re-start collection
          }
        }

        // If no jump or if it's the first point, add it to buffer if space available
        if (initial_mapping_points_buffer_.size() < fixed_alignment_buffer_size_) {
          initial_mapping_points_buffer_.push_back(current_mapping_pt);
          last_initial_mapping_point_ = current_mapping_pt; // Store this point for next comparison
          first_initial_mapping_point_ = false; // Mark that we've processed at least one point
        } else {
            // Buffer is full, stop adding points from this message and wait for NavSatFix to trigger calculation
            break;
        }
      }
      RCLCPP_DEBUG(this->get_logger(), "Collecting initial Mapping points: %zu/%zu", initial_mapping_points_buffer_.size(), fixed_alignment_buffer_size_);
    }
  }

  /**
   * @brief Converts RTK GPS status integer to a ColorRGBA message.
   * @param status The integer status code from NavSatStatus.
   * @return A std_msgs::msg::ColorRGBA representing the color for the given status.
   */
  std_msgs::msg::ColorRGBA rtkStatusToColor(int status) {
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0; // Fully opaque
    switch (status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
        color.r = 1.0; color.g = 1.0; color.b = 1.0; // White
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
        color.r = 1.0; color.g = 0.0; color.b = 0.0; // Red
        break;
      case 5: // RTK Float (as per Emlid definition)
        color.r = 1.0; color.g = 1.0; color.b = 0.0; // Yellow
        break;
      case 4: // RTK FIX (as per Emlid definition)
        color.r = 0.0; color.g = 1.0; color.b = 0.0; // Green
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Invalid RTK Status: %d. Defaulting to grey.", status);
        color.r = 0.5; color.g = 0.5; color.b = 0.5; // Grey for unknown status
    }
    return color;
  }

  /**
   * @brief Saves the accumulated colored path markers to a PCD file.
   * Each point from each LINE_STRIP marker is saved with its color.
   * @param filename The path and name of the PCD file to save.
   */
  void savePathMarkersToPCD(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& marker_array_element : colored_path_marker_array_.markers) {
        if (marker_array_element.type == visualization_msgs::msg::Marker::LINE_STRIP) {
            for (const auto& pt : marker_array_element.points) {
                pcl::PointXYZRGB pcl_point;
                pcl_point.x = pt.x;
                pcl_point.y = pt.y;
                pcl_point.z = pt.z;

                // Convert float color (0.0-1.0) to uint8 (0-255) and pack into RGB
                uint8_t r = static_cast<uint8_t>(marker_array_element.color.r * 255);
                uint8_t g = static_cast<uint8_t>(marker_array_element.color.g * 255);
                uint8_t b = static_cast<uint8_t>(marker_array_element.color.b * 255);
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                                static_cast<uint32_t>(g) << 8 |
                                static_cast<uint32_t>(b));
                pcl_point.rgb = *reinterpret_cast<float*>(&rgb); // PCL stores RGB as float

                cloud->points.push_back(pcl_point);
            }
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1; // Unordered point cloud
    cloud->is_dense = true; // No NaN points

    if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved %zu colored path points to %s", cloud->size(), filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file to %s", filename.c_str());
    }
  }

  /**
   * @brief Saves the accumulated covariance information as principal axes (lines) to a PLY file.
   * @param filename The path and name of the PLY file to save.
   */
  void saveCovarianceAsLinesToPLY(const std::string& filename) {
      std::ofstream file(filename);
      if (!file.is_open()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to open PLY file: %s", filename.c_str());
          return;
      }

      std::vector<pcl::PointXYZRGB> ply_points;
      std::vector<std::array<int, 2>> ply_lines; // Pairs of vertex indices for lines

      double sigma_multiplier = 2.0; // Show 2-sigma extent of the ellipsoid axes

      for (const auto& cov_data : covariance_data_for_ply_) {
          // Reconstruct Eigen Matrix from the std::array
          Eigen::Matrix3d cov_eigen;
          cov_eigen << cov_data.covariance_matrix[0], cov_data.covariance_matrix[1], cov_data.covariance_matrix[2],
                       cov_data.covariance_matrix[3], cov_data.covariance_matrix[4], cov_data.covariance_matrix[5],
                       cov_data.covariance_matrix[6], cov_data.covariance_matrix[7], cov_data.covariance_matrix[8];

          // Perform eigenvalue decomposition
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov_eigen);
          if (es.info() != Eigen::Success) {
              RCLCPP_WARN(this->get_logger(), "Eigen decomposition failed for a covariance matrix. Skipping this point.");
              continue;
          }

          Eigen::Vector3d eigenvalues = es.eigenvalues();
          Eigen::Matrix3d eigenvectors = es.eigenvectors();

          pcl::PointXYZRGB center_point_pcl;
          center_point_pcl.x = cov_data.position.x;
          center_point_pcl.y = cov_data.position.y;
          center_point_pcl.z = cov_data.position.z;
          uint8_t r = static_cast<uint8_t>(cov_data.color.r * 255);
          uint8_t g = static_cast<uint8_t>(cov_data.color.g * 255);
          uint8_t b = static_cast<uint8_t>(cov_data.color.b * 255);
          uint32_t rgb_packed = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
          center_point_pcl.rgb = *reinterpret_cast<float*>(&rgb_packed);

          int center_idx = ply_points.size();
          ply_points.push_back(center_point_pcl);

          for (int i = 0; i < 3; ++i) { // For each of the three principal axes
              // Ensure eigenvalue is non-negative before sqrt due to potential numerical precision issues
              double std_dev = std::sqrt(std::max(0.0, eigenvalues(i)));
              Eigen::Vector3d axis_direction = eigenvectors.col(i);

              // Calculate point in positive direction along axis
              Eigen::Vector3d p1_vec = Eigen::Vector3d(center_point_pcl.x, center_point_pcl.y, center_point_pcl.z) + sigma_multiplier * std_dev * axis_direction;
              pcl::PointXYZRGB p1_pcl;
              p1_pcl.x = p1_vec.x(); p1_pcl.y = p1_vec.y(); p1_pcl.z = p1_vec.z();
              p1_pcl.rgb = center_point_pcl.rgb; // Use the same color as the center point
              int p1_idx = ply_points.size();
              ply_points.push_back(p1_pcl);
              ply_lines.push_back({center_idx, p1_idx}); // Line from center to positive end

              // Calculate point in negative direction along axis
              Eigen::Vector3d p2_vec = Eigen::Vector3d(center_point_pcl.x, center_point_pcl.y, center_point_pcl.z) - sigma_multiplier * std_dev * axis_direction;
              pcl::PointXYZRGB p2_pcl;
              p2_pcl.x = p2_vec.x(); p2_pcl.y = p2_vec.y(); p2_pcl.z = p2_vec.z();
              p2_pcl.rgb = center_point_pcl.rgb; // Use the same color as the center point
              int p2_idx = ply_points.size();
              ply_points.push_back(p2_pcl);
              ply_lines.push_back({center_idx, p2_idx}); // Line from center to negative end
          }
      }

      // Write PLY header
      file << "ply\n";
      file << "format ascii 1.0\n"; // ASCII format for readability; can change to binary
      file << "element vertex " << ply_points.size() << "\n";
      file << "property float x\n";
      file << "property float y\n";
      file << "property float z\n";
      file << "property uchar red\n";
      file << "property uchar green\n";
      file << "property uchar blue\n";
      file << "element edge " << ply_lines.size() << "\n";
      file << "property list uchar int vertex_indices\n"; // Format for edges: (num_vertices, v1_idx, v2_idx, ...)
      file << "end_header\n";

      // Write vertex data
      for (const auto& p : ply_points) {
          file << p.x << " " << p.y << " " << p.z << " ";
          uint32_t rgb_val = *reinterpret_cast<const uint32_t*>(&p.rgb);
          uint8_t r = (rgb_val >> 16) & 0xFF;
          uint8_t g = (rgb_val >> 8) & 0xFF;
          uint8_t b = rgb_val & 0xFF;
          file << static_cast<int>(r) << " " << static_cast<int>(g) << " " << static_cast<int>(b) << "\n";
      }

      // Write edge data
      for (const auto& l : ply_lines) {
          file << "2 " << l[0] << " " << l[1] << "\n"; // '2' means each edge connects 2 vertices
      }

      file.close();
      RCLCPP_INFO(this->get_logger(), "Saved %zu covariance line segments to %s", ply_lines.size(), filename.c_str());
  }


  /**
   * @brief Service callback to trigger saving of all accumulated GPS data to PCD files.
   * @param request Shared pointer to the Trigger service request.
   * @param response Shared pointer to the Trigger service response.
   */
  void saveGpsDataToPCDService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void) request; // Suppress unused parameter warning

    std::string path_filename = "/gps_path_points_service.pcd";
    std::string covariance_lines_filename = "/gps_covariance_lines_service.ply";
    try {
      savePathMarkersToPCD(path_filename);
      saveCovarianceAsLinesToPLY(covariance_lines_filename);
      response->success = true;
      response->message = "Saved path points to " + path_filename + " and covariance lines to " + covariance_lines_filename;
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Failed to save data: " + std::string(e.what());
    }
  }

  // ROS 2 Subscribers and Publishers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_navsatfix_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_mapping_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_colored_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_covariance_markers_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_service_;

  // GeographicLib for coordinate transformations
  GeographicLib::LocalCartesian local_cartesian_;

  // Data structures for path and covariance markers
  visualization_msgs::msg::MarkerArray colored_path_marker_array_; // Stores multiple LINE_STRIP markers for the path
  int line_marker_id_counter_ = 0; // Separate ID counter for path markers
  int current_line_marker_id_ = -1; // To keep track of the current line segment to append to
  int last_fix_status_ = -1; // To detect changes in fix status for path coloring

  visualization_msgs::msg::MarkerArray covariance_marker_array_; // Stores all covariance markers (for RViz CUBE)
  int marker_id_counter_ = 0; // ID counter for covariance markers

  // Parameters for marker scaling
  float marker_scale_covariance_; // Scale parameter for covariance markers (for RViz CUBE)
  float line_strip_scale_; // For the thickness of the path line

  // Members for fixed Z-axis alignment
  std::deque<geometry_msgs::msg::Point> initial_rtk_points_buffer_; // Buffer for initial RTK points
  std::deque<geometry_msgs::msg::Point> initial_mapping_points_buffer_; // Buffer for initial mapping path points
  size_t fixed_alignment_buffer_size_; // Size N for the initial alignment buffers
  bool initial_offset_calculated_ = false; // Flag to indicate if the fixed offset has been calculated
  double fixed_z_offset_ = 0.0; // The single, fixed Z-offset to be applied to all subsequent points

  // Members for initial jump detection (only for mapping path)
  geometry_msgs::msg::Point last_initial_mapping_point_; // Stores the previous good mapping point for jump comparison
  bool first_initial_mapping_point_ = true; // Flag to indicate the very first point in the mapping collection
  double max_initial_xy_jump_; // Threshold for detecting large XY jumps in meters

  // Data storage for PLY export of covariance (full matrix)
  struct CovarianceData {
      geometry_msgs::msg::Point position;
      std::array<double, 9> covariance_matrix; // Row-major: C[0,0], C[0,1] ... C[2,2]
      std_msgs::msg::ColorRGBA color;
  };
  std::vector<CovarianceData> covariance_data_for_ply_;
};

/**
 * @brief Main function to initialize and run the ROS 2 node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<EmlidRtkColor>("emlid_rtk_color", "emlid_color", options));
  rclcpp::shutdown();
  return 0;
}