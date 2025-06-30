#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include <CppLinuxSerial/SerialPort.hpp>
#include <thread>
#include <atomic>
#include <vector>

using namespace mn::CppLinuxSerial;

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

  EmlidGps(const std::string & name,
    const std::string & ns,
    const rclcpp::NodeOptions & options)
  : Node(name, ns, options),
  serialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE),
  stop_flag_(false)
  {

    // Set up parameters
    this->declare_parameter("origin_lat", 40.246266148);
    this->declare_parameter("origin_lon", -111.648571401);
    this->declare_parameter("origin_alt", 1403.8386);
    this->declare_parameter("frame_id", "map");
    double origin_lat, origin_lon, origin_alt;
    
    this->get_parameter("origin_lat", origin_lat);
    this->get_parameter("origin_lon", origin_lon);
    this->get_parameter("origin_alt", origin_alt);
    this->get_parameter("frame_id", frame_id);

    geo_converter_ = std::make_unique<GeographicLib::LocalCartesian>(origin_lat, origin_lon, origin_alt);
    path_.header.frame_id = frame_id;

    // Publishers
    publisher_raw_ = this->create_publisher<std_msgs::msg::String>("nmea", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("NavSatFix", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_rel", 10);
  
    // Serial port setup
    serialPort.SetTimeout(100);
    try
    {
      serialPort.Open();
    } 
    catch (const std::exception &e) 
    {
      RCLCPP_ERROR(this->get_logger(), "%s\nShutting Down", e.what());
      rclcpp::shutdown();
    }

    read_thread_ = std::thread(&EmlidGps::readLoop, this);
  }

  ~EmlidGps(){
    stop_flag_ = true;
    if (read_thread_.joinable()) 
    {
      read_thread_.join();
    }
    serialPort.Close();
  }

  void readLoop()
  {
    while (rclcpp::ok() && !stop_flag_)
    {
      try 
      {
        std::string data = "";
        serialPort.Read(data);
        if (!data.empty() && data[0] == '$')
        {
          RCLCPP_DEBUG(this->get_logger(), "Raw sentence: \n%s", data.c_str());
          std_msgs::msg::String msg_raw;
          msg_raw.data = data;
          
          // Parse Data for NavSatFix and Relative Location
          size_t start = 0;
          size_t end = data.find("\r\n");
          sensor_msgs::msg::NavSatFix fix_msg;
          rclcpp::Time gps_time;

          bool do_publish = true;

          while (end != std::string::npos)
          {
            std::string sentence = data.substr(start, end-start);
            if (!sentence.empty() && sentence[0] == '$')
            {
              std_msgs::msg::String msg;
              msg.data = sentence;
              
              std::stringstream ss(sentence);
              std::string field;
              std::vector<std::string> fields;
              
              while (std::getline(ss, field, ',')) 
              {
                fields.push_back(field);
              }

              switch (getNmeaType(fields[0]))
              {
                case NmeaType::GGA:
                  if (!EmlidGps::parseGGA(fields, fix_msg))
                  {
                    RCLCPP_WARN(this->get_logger(), "Parsing of GGA sentence failed: %s", sentence.c_str());
                    do_publish = false;
                  }
                  break;
                case NmeaType::RMC:
                  if (!EmlidGps::parseRMC(fields, gps_time))
                  {
                    RCLCPP_WARN(this->get_logger(), "Parsing of RMC sentence failed: %s", sentence.c_str());
                    do_publish = false;
                  }
                  break;
                case NmeaType::GST:
                  if (!EmlidGps::parseGST(fields, fix_msg))
                  {
                    RCLCPP_WARN(this->get_logger(), "Parsing of GST sentence failed: %s", sentence.c_str());
                    do_publish = false;
                  }
                  break;
                case NmeaType::UNKNOWN:
                  RCLCPP_DEBUG(this->get_logger(), "Got unknown/unimplemented NMEA type: %s", sentence.c_str());
                  break;
                default:
                  RCLCPP_WARN(this->get_logger(), "Unknown NMEA case");
                  break;
                
              }
            }
            start = end + 2;  // Skip past "\r\n"
            end = data.find("\r\n", start);
          }

          fix_msg.header.stamp = gps_time;
          fix_msg.header.frame_id = "gps_link";

          publisher_raw_->publish(msg_raw);
          
          if (do_publish)
          {
            gps_pub_->publish(fix_msg);
            // Convert lat, lon, alt to relative position and publish Ground Truth Path
            double x_e, y_n, z_u;
            geo_converter_->Forward(fix_msg.latitude, fix_msg.longitude, fix_msg.altitude, x_e, y_n, z_u);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = fix_msg.header.stamp;
            pose.header.frame_id = frame_id;
            
            pose.pose.position.x = x_e;
            pose.pose.position.y = y_n;
            pose.pose.position.z = z_u;
            
            pose.pose.orientation.w = 1.0; // No rotation
            
            // Check if there are any subscribers
            if (path_pub_->get_subscription_count() > 0) 
            {
              path_.poses.push_back(pose);
              path_.header.stamp = fix_msg.header.stamp;
              path_pub_->publish(path_);
            } 
            else 
            {
              // No subscribers — clear the path
              path_.poses.clear();
            }
          }
          
        }
      } catch (const std::exception &e) 
      {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", e.what());
      }
    }
  }
  
private:

  NmeaType getNmeaType(const std::string & type_str)
  {
    std::string sub_str = type_str.substr(3); // Removes first three characters
    if (sub_str == "GGA") return NmeaType::GGA;
    // else if (sub_str == "GSA")
    // {
    //   if (type_str == "GPGSA") return NmeaType::GPGSA;
    //   if (type_str == "GLGSA") return NmeaType::GLGSA;
    //   if (type_str == "GAGSA") return NmeaType::GAGSA;
    //   if (type_str == "GLGSA") return NmeaType::GLGSA;
    // }
    else if (sub_str == "GST") return NmeaType::GST;
    else if (sub_str == "RMC") return NmeaType::RMC;
    else return NmeaType::UNKNOWN;
  }

  double convertNMEACoordinate(const std::string& nmea_coord, const std::string& direction) {
    if (nmea_coord.empty()) return 0.0;
  
    size_t dot = nmea_coord.find('.');
    int deg_len = (dot > 2) ? dot - 2 : 2;
    double degrees = std::stod(nmea_coord.substr(0, deg_len));
    double minutes = std::stod(nmea_coord.substr(deg_len));
    double decimal = degrees + minutes / 60.0;
  
    if (direction == "S" || direction == "W")
      decimal *= -1.0;
  
    return decimal;
  }
  
  bool parseGGA(const std::vector<std::string>& fields, sensor_msgs::msg::NavSatFix& msg) 
  {

    if (fields.size() < 10) return false;
  
    msg.latitude = convertNMEACoordinate(fields[2], fields[3]);
    msg.longitude = convertNMEACoordinate(fields[4], fields[5]);
    msg.altitude = std::stod(fields[9]);

    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  
    return true;
  }
  
  bool parseRMC(const std::vector<std::string>& fields, rclcpp::Time& ros_time_out) 
  {

    if (fields.size() < 10 || fields[2] != "A") return false;
  
    std::string time_str = fields[1];   // e.g. "174628.80"
    std::string date_str = fields[9];   // e.g. "240625" (DDMMYY)
  
    if (time_str.length() < 6 || date_str.length() != 6) return false;
  
    int hours   = std::stoi(time_str.substr(0, 2));
    int minutes = std::stoi(time_str.substr(2, 2));
    int seconds = std::stoi(time_str.substr(4, 2));
    int millis  = 0;
    if (time_str.length() > 6 && time_str[6] == '.') {
      millis = std::stoi(time_str.substr(7, 2));
    }
  
    int day   = std::stoi(date_str.substr(0, 2));
    int month = std::stoi(date_str.substr(2, 2));
    int year  = std::stoi(date_str.substr(4, 2)) + 2000;
  
    std::tm t{};
    t.tm_year = year - 1900;
    t.tm_mon  = month - 1;
    t.tm_mday = day;
    t.tm_hour = hours;
    t.tm_min  = minutes;
    t.tm_sec  = seconds;
    time_t unix_time = timegm(&t);  // Use `timegm` for UTC
  
    ros_time_out = rclcpp::Time(unix_time, millis * 1e6); // nanoseconds
  
    return true;
  }

  bool parseGST(const std::vector<std::string>& fields, sensor_msgs::msg::NavSatFix& msg)
  {
    if (fields.size() < 9) return false;

    try 
    {
      RCLCPP_INFO(this->get_logger(), "lat std: %s, lon std: %s, alt std: %s", 
                                      fields[6].c_str(), fields[7].c_str(), fields[8].c_str());
      double lat_std = std::stod(fields[6]);
      double lon_std = std::stod(fields[7]);
      // Remove checksum part from alt_std
      std::string alt_field = fields[8];
      size_t asterisk_pos = alt_field.find('*');
      if (asterisk_pos != std::string::npos) {
          alt_field = alt_field.substr(0, asterisk_pos);
      }
      double alt_std = std::stod(alt_field);

      msg.position_covariance[0] = lon_std * lon_std; // E
      msg.position_covariance[4] = lat_std * lat_std; // N
      msg.position_covariance[8] = alt_std * alt_std; // U

      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(this->get_logger(), "Error with converting GST fields: %s", e.what());
    }

    return true;

  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_raw_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_;
  std::unique_ptr<GeographicLib::LocalCartesian> geo_converter_;
  SerialPort serialPort;
  std::thread read_thread_;
  std::atomic<bool> stop_flag_;
  std::string frame_id;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // options.arguments(std::vector<std::string>{});
  // options.set_namespace("emlid");
  rclcpp::spin(std::make_shared<EmlidGps>("emlid_gps", "emlid", options));
  rclcpp::shutdown();
  return 0;
}
