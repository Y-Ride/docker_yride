#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <vector>
#include <fstream>
#include <filesystem>


class TimestampsToTxt : public rclcpp::Node 
{
  public:

  enum class TopicType { IMU, LIDAR_IMU };

  TimestampsToTxt() : Node("timestamps_to_txt")
  {
    RCLCPP_INFO(rclcpp::get_logger("start"), "TimestampsToTxt node started");

    // Declare parameters
    this->declare_parameter<std::string>("output_folder", "./");
    this->declare_parameter<std::string>("lidar_topic", "/ouster/points");
    this->declare_parameter<std::string>("imu_topic", "/imu");
    this->declare_parameter<std::string>("lidar_imu_topic", "");

    // Get parameters
    output_folder               = this->get_parameter("output_folder").as_string();
    std::string lidar_topic     = this->get_parameter("lidar_topic").as_string();
    std::string lidar_imu_topic = this->get_parameter("lidar_imu_topic").as_string();
    bool lidar_imu_topic_set    = !lidar_imu_topic.empty();
    std::string imu_topic       = this->get_parameter("imu_topic").as_string();

    // Check if output_folder exists and make it if it doesn't
    if (!std::filesystem::exists(output_folder))
    {
      RCLCPP_INFO(rclcpp::get_logger("OutputFolder"), "No output folder exists. Creating one at %s", output_folder.c_str());
      std::filesystem::create_directories(output_folder);
    }

    // Print parameters to make sure I got them correctly
    RCLCPP_INFO(rclcpp::get_logger("Parameters"), "Output folder: %s", output_folder.c_str());
    RCLCPP_INFO(rclcpp::get_logger("Parameters"), "Lidar points topic: %s", lidar_topic.c_str());
    if (lidar_imu_topic_set) 
      RCLCPP_INFO(rclcpp::get_logger("Parameters"), "Lidar imu topic: %s", lidar_imu_topic.c_str());
    else 
      RCLCPP_WARN(rclcpp::get_logger("Parameters"), "Lidar IMU topic is not set");
    RCLCPP_INFO(rclcpp::get_logger("Parameters"), "Imu topic: %s", imu_topic.c_str());

    // Initialize subscribers

    subPointCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, 100, std::bind(&TimestampsToTxt::pointCloudCallback, this, std::placeholders::_1)
    );
    subImu = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 100, 
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { // Lambda function
        imuCallback(msg, TopicType::IMU);
      }
      //std::bind(&TimestampsToTxt::imuCallback, this, std::placeholders::_1, TopicType::IMU)
    );
    if (lidar_imu_topic_set)
    {
      subLidarImu = this->create_subscription<sensor_msgs::msg::Imu>(
        lidar_imu_topic, 100, 
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { // Lambda function
          imuCallback(msg, TopicType::LIDAR_IMU);
        }
        //std::bind(&TimestampsToTxt::imuCallback, this, std::placeholders::_1, TopicType::LIDAR_IMU)
      );
    }
    RCLCPP_INFO(rclcpp::get_logger("init"), "Subscribers initialized");

  }

  ~TimestampsToTxt()
  {
    RCLCPP_INFO(rclcpp::get_logger("destructor"), "Writing information to files");
    writeTimestampsToFile();
  }

  private:
  std::string output_folder;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointCloud;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subLidarImu;
  std::vector<rclcpp::Time> pointCloudTimestamps;
  std::vector<rclcpp::Time> imuTimestamps;
  std::vector<geometry_msgs::msg::Vector3> imuData;
  std::vector<rclcpp::Time> lidarImuTimestamps;
  std::vector<geometry_msgs::msg::Vector3> lidarImuData;
  bool first_pc = true;
  bool first_imu = true;
  bool first_lidarImu = true;
  uint32_t pc_count = 0;
  uint32_t imu_count = 0;
  uint32_t lidarImu_count = 0;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  { 
    if (first_pc){
      RCLCPP_INFO(rclcpp::get_logger("Pointcloud Callback"), "Received pointcloud message");
      RCLCPP_INFO(rclcpp::get_logger("Pointcloud Callback"), "Timestamp: %d.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
      first_pc = false;
    }
    pointCloudTimestamps.push_back(msg->header.stamp);
    pc_count++;
    if (pc_count % 10 == 0){
      RCLCPP_INFO(rclcpp::get_logger("Pointcloud Callback"), "Received %d pointcloud messages", pc_count);
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, TopicType topic)
  {
    if (topic == TopicType::IMU)
    {
      if (first_imu){
        RCLCPP_INFO(rclcpp::get_logger("IMU Callback"), "Received IMU message");
        RCLCPP_INFO(rclcpp::get_logger("IMU Callback"), "Timestamp: %d.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
        first_imu = false;
      }

      imuTimestamps.push_back(msg->header.stamp);
      imuData.push_back(msg->linear_acceleration);
      imu_count++;

      if (imu_count % 100 == 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("IMU Callback"), "Received %d IMU messages", imu_count);
      }
    } 
    else if (topic == TopicType::LIDAR_IMU)
    {
      if (first_lidarImu)
      {
        RCLCPP_INFO(rclcpp::get_logger("Lidar IMU Callback"), "Recieved Lidar IMU data");
        first_lidarImu = false;
      }
      
      lidarImuTimestamps.push_back(msg->header.stamp);
      lidarImuData.push_back(msg->linear_acceleration);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("IMU Callback"), "Invalid TopicType Value!");
    }
  }

  void writeTimestampsToFile()
  {
    // Write Lidar Data
    FILE *fp;
    fp = fopen((output_folder+"timestamps_lidar.txt").c_str(), "w");

    if (fp == NULL)
    {
      RCLCPP_ERROR(rclcpp::get_logger("writeTimestampsToFile"), "Error opening Lidar Pointcloud file");
      return;
    }

    fprintf(fp, "Time in seconds,Time in nanoseconds\n");

    for (size_t i = 0; i < pointCloudTimestamps.size(); i++)
    {
      fprintf(fp, "%f,%ld\n", pointCloudTimestamps[i].seconds(), pointCloudTimestamps[i].nanoseconds());
    }
    fclose(fp);

    // Write IMU data
    fp = fopen((output_folder+"timestamps_imu.txt").c_str(), "w");

    if (fp == NULL)
    {
      RCLCPP_ERROR(rclcpp::get_logger("writeTimestampsToFile"), "Error opening Imu file");
      return;
    }

    std::string imu_title = "Time in seconds,Time in nanoseconds,linear_x,linear_y,linear_z";
    fprintf(fp, "%s\n", imu_title.c_str());

    for (size_t i = 0; i < imuTimestamps.size(); i++)
    {
      fprintf(fp, "%f,%ld,%lf,%lf,%lf\n", imuTimestamps[i].seconds(), imuTimestamps[i].nanoseconds(),
              imuData[i].x, imuData[i].y, imuData[i].z);
    }
    fclose(fp);

    // Write Imu data from the lidar imu if it is available
    if (lidarImuTimestamps.size() > 0)
    {
      fp = fopen((output_folder+"timestamps_lidarImu.txt").c_str(), "w");

      if (fp == NULL)
      {
        RCLCPP_ERROR(rclcpp::get_logger("writeTimestampsToFile"), "Error opening Lidar Imu file");
        return;
      }

      fprintf(fp, "%s\n", imu_title.c_str()); // Header
      
      for (size_t i = 0; i < lidarImuTimestamps.size(); i++)
      {
        fprintf(fp, "%f,%ld,%lf,%lf,%lf\n", lidarImuTimestamps[i].seconds(), lidarImuTimestamps[i].nanoseconds(),
              lidarImuData[i].x, lidarImuData[i].y, lidarImuData[i].z);
      }   
      fclose(fp);
    }

    RCLCPP_INFO(rclcpp::get_logger("writeTimestampsToFile"), "All files successfully written!");
  }


};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("Timestamps_to_txt_node\n");

  rclcpp::init(argc, argv);

  auto node = std::make_shared<TimestampsToTxt>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
