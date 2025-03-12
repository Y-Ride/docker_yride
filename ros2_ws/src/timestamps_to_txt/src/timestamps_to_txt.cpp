#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>
#include <fstream>


class TimestampsToTxt : public rclcpp::Node 
{
  public:
  TimestampsToTxt() : Node("timestamps_to_txt")
  {
    RCLCPP_INFO(rclcpp::get_logger("start"), "TimestampsToTxt node started");

    // Initialize subscribers

    subPointCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster_top/points", 100, std::bind(&TimestampsToTxt::pointCloudCallback, this, std::placeholders::_1)
    );
    subImu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/inertial_sense/imu_raw", 100, std::bind(&TimestampsToTxt::imuCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(rclcpp::get_logger("init"), "Subscribers initialized");

    // // Add Shutdown Handler
    // this->add_on_shutdown_callback([this]() {
    //   RCLCPP_INFO(rclcpp::get_logger("shutdown"), "Shutting down node");
    //   writeTimestampsToFile();
    // });


  }

  ~TimestampsToTxt()
  {
    RCLCPP_INFO(rclcpp::get_logger("destructor"), "Destructing node");
    writeTimestampsToFile();
  }

  private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointCloud;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
  std::vector<rclcpp::Time> pointCloudTimestamps;
  std::vector<rclcpp::Time> imuTimestamps;
  bool first_pc = true;
  bool first_imu = true;
  uint32_t pc_count = 0;
  uint32_t imu_count = 0;

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

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (first_imu){
      RCLCPP_INFO(rclcpp::get_logger("IMU Callback"), "Received IMU message");
      RCLCPP_INFO(rclcpp::get_logger("IMU Callback"), "Timestamp: %d.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
      first_imu = false;
    }
    imuTimestamps.push_back(msg->header.stamp);
    imu_count++;
    if (imu_count % 100 == 0){
      RCLCPP_INFO(rclcpp::get_logger("IMU Callback"), "Received %d IMU messages", imu_count);
    }
  }

  void writeTimestampsToFile()
  {
    FILE *fp;
    fp = fopen("timestamps_lidar.txt", "w");
    if (fp == NULL)
    {
      RCLCPP_ERROR(rclcpp::get_logger("writeTimestampsToFile"), "Error opening file");
      return;
    }
    fprintf(fp, "Time in seconds, Time in nanoseconds\n");
    for (size_t i = 0; i < pointCloudTimestamps.size(); i++)
    {
      fprintf(fp, "%f, %ld\n", pointCloudTimestamps[i].seconds(), pointCloudTimestamps[i].nanoseconds());
    }
    fclose(fp);

    fp = fopen("timestamps_imu.txt", "w");
    if (fp == NULL)
    {
      RCLCPP_ERROR(rclcpp::get_logger("writeTimestampsToFile"), "Error opening file");
      return;
    }
    fprintf(fp, "Time in seconds, Time in nanoseconds\n");
    for (size_t i = 0; i < imuTimestamps.size(); i++)
    {
      fprintf(fp, "%f, %ld\n", imuTimestamps[i].seconds(), imuTimestamps[i].nanoseconds());
    }
    fclose(fp);
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
