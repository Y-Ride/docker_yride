#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <CppLinuxSerial/SerialPort.hpp>
#include <thread>
#include <atomic>
#include <vector>

using namespace mn::CppLinuxSerial;

class EmlidGps : public rclcpp::Node
{
public:
  EmlidGps(): Node("emlid_gps"),
  serialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE),
  stop_flag_(false)
  {
    publisher_raw_ = this->create_publisher<std_msgs::msg::String>("emlid_nmea", 10);
  
    // Serial port setup
    serialPort.SetTimeout(100);
    try{
      serialPort.Open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "%s\nShutting Down", e.what());
      rclcpp::shutdown();
    }

    read_thread_ = std::thread(&EmlidGps::readLoop, this);
  }
  ~EmlidGps(){
    stop_flag_ = true;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    serialPort.Close();
  }

  void readLoop()
  {
    while (rclcpp::ok() && !stop_flag_)
    {
      try {
        std::string data = "";
        serialPort.Read(data);
        if (!data.empty())
        {
          std_msgs::msg::String msg_raw;
          msg_raw.data = data;
          publisher_raw_->publish(msg_raw);

          // Parse Data for NavSatFix and Relative Location
          size_t start = 0;
          size_t end = data.find("\r\n");
          sensor_msgs::msg::NavSatFix fix_msg;
          rclcpp::Time gps_time;
          bool do_publish = false;
          while (end != std::string::npos)
          {
            std::string sentence = data.substr(start, end-start);
            if (!sentence.empty())
            {
              std_msgs::msg::String msg;
              msg.data = sentence;
              std::cout << "Start = " << start << ", Sentence = \"" << sentence << "\"" << std::endl;

              std::stringstream ss(sentence);
              std::string field;
              std::vector<std::string> fields;
  
              while (std::getline(ss, field, ',')) {
                fields.push_back(field);
              }

              if (fields[0] == "$GNGGA"){
                if (!EmlidGps::parseGNGGA(fields, fix_msg))
                {
                  do_publish = false;
                }
              }
              else if (fields[0] == "$GNRMC") {
                if (!EmlidGps::parseGNRMC(fields, gps_time))
                {
                  do_publish = false;
                }
              }
              else {
                RCLCPP_WARN(this->get_logger(), "Invalid nmea sentence type! Sentence was \"%s\"", sentence.c_str());
              }
            }
            start = end + 2;  // Skip past "\r\n"
            end = data.find("\r\n", start);
          }
          fix_msg.header.stamp = gps_time;
          fix_msg.header.frame_id = "gps_link";
          if (do_publish)
          {  
            gps_pub_enu_->publish(fix_msg);
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", e.what());
      }
    }
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
  
  bool parseGNGGA(const std::vector<std::string>& fields, sensor_msgs::msg::NavSatFix& msg) {
    
  
    if (fields.size() < 10) return false;
  
    msg.latitude = convertNMEACoordinate(fields[2], fields[3]);
    msg.longitude = convertNMEACoordinate(fields[4], fields[5]);
    msg.altitude = std::stod(fields[9]);

    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  
    return true;
  }
  
  bool parseGNRMC(const std::vector<std::string>& fields, rclcpp::Time& ros_time_out) {
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

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_raw_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_enu_;
  SerialPort serialPort;
  std::thread read_thread_;
  std::atomic<bool> stop_flag_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmlidGps>());
  rclcpp::shutdown();
  return 0;
}
