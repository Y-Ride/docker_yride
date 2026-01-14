#pragma once

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include <CppLinuxSerial/SerialPort.hpp>
#include <thread>
#include <atomic>
#include <vector>


class EmlidGps : public rclcpp::Node
{
public:

  enum class NmeaType
  {
    GGA,                            // Time, position and fix type data
    GPGSA, GLGSA, GAGSA, GBGSA,     // GPS receiver operating mode, satellites used in the position solution, and DOP values
    GST,                            // Position error statistics
    GPGSV, GLGSV, GAGSV, GBGSV,     // Number of satellites in view, ID numbers, elevation, azimuth, & SNR values
    RMC,                            // Position, velocity, and time
    VTG,                            // Course and speed information relative to the ground
    ZDA,                            // UTC day, month, and year, and local time zone offset
    EBP,                            // This is a custom Emlid NMEA message. RTK base position
    UNKNOWN
  };

  EmlidGps(const std::string & name, const std::string & ns, const rclcpp::NodeOptions & options);
  ~EmlidGps();
  void readLoop();
  void ResetSerialPort();

  static constexpr int8_t STATUS_RTK_FIXED = 4;
  static constexpr int8_t STATUS_RTK_FLOAT = 5;
  
  private:
  
  // Functions
  NmeaType getNmeaType(const std::string & type_str);
  double convertNMEACoordinate(const std::string& nmea_coord, const std::string& direction);
  bool parseGGA(const std::vector<std::string>& fields, sensor_msgs::msg::NavSatFix& msg);
  bool parseRMC(const std::vector<std::string>& fields, rclcpp::Time& ros_time_out);
  bool parseGST(const std::vector<std::string>& fields, sensor_msgs::msg::NavSatFix& msg);
  
  // Variables
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_raw_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_;
  std::unique_ptr<GeographicLib::LocalCartesian> geo_converter_;
  mn::CppLinuxSerial::SerialPort serialPort;
  std::thread read_thread_;
  std::atomic<bool> stop_flag_;
  std::string frame_id;
  int previous_fix_status_ = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  rclcpp::Time last_fix_status_change_time_ = this->now();


};
