#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>    // Required for std::vector
#include <iomanip>   // Required for std::setfill and std::setw
#include <cstring>   // Required for std::memcpy
#include <algorithm> // Required for std::min

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/time.hpp"     // Required for rclcpp::Time
#include "rclcpp/duration.hpp" // Required for rclcpp::Duration

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename, const std::string & topic)
    : Node("read_timestamps_ouster")
    {
      // Create a publisher for the /test/points topic
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points", 10);
      // Create a wall timer to trigger the timer_callback function every 100ms
      timer_ = this->create_wall_timer(
          100ms, std::bind(&PlaybackNode::timer_callback, this));
      
      topic_ = topic;
      // Configure storage options for the rosbag2 reader
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_filename; // Set the bag file path
      // Create a rosbag2 reader instance
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      // Open the bag file
      reader_->open(storage_options);
    }

  private:
    void timer_callback()
    {
      // Loop as long as there are messages in the bag
      while (reader_->has_next()) {
        // Read the next serialized bag message
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

        // Skip messages that are not from the target topic
        if (msg->topic_name != topic_) {
          continue;
        }

        // Create a serialized message object from the bag message's serialized data
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        // Create a shared pointer for the PointCloud2 message
        sensor_msgs::msg::PointCloud2::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Deserialize the message into the PointCloud2 object
        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        // Publish the deserialized PointCloud2 message
        publisher_->publish(*ros_msg);

        // Print the header timestamp and frame_id of the PointCloud2 message
        // std::setfill('0') and std::setw(9) are used to format nanoseconds with leading zeros
        std::cout << "Header Timestamp: (" << ros_msg->header.stamp.sec << "."
                  << std::setfill('0') << std::setw(9) << ros_msg->header.stamp.nanosec
                  << ", frame_id: " << ros_msg->header.frame_id << ")\n";

        // --- Start of new logic for extracting column timestamps ---

        int t_offset = -1; // Initialize offset for 't' field
        // Iterate through the fields to find the 't' (timestamp) field
        for (const auto& field : ros_msg->fields) {
            if (field.name == "t") {
                t_offset = field.offset; // Store the offset of the 't' field
                break;
            }
        }

        // Check if the 't' field was found
        if (t_offset == -1) {
            std::cerr << "Error: 't' (timestamp) field not found in PointCloud2 message. Skipping timestamp extraction for this message." << std::endl;
            break; // Exit the while loop to process the next message on the next timer tick
        }

        // Create a rclcpp::Time object from the message header's timestamp
        rclcpp::Time header_time(ros_msg->header.stamp);

        // Determine how many columns to print (up to 10, or fewer if the PointCloud2 width is smaller)
        int columns_to_print = std::min((int)ros_msg->width, 10);

        std::cout << "First " << columns_to_print << " column timestamps (relative to header):\n";
        // Iterate through the desired number of columns
        for (int j = 0; j < columns_to_print; ++j) {
            // Calculate the starting byte offset for the first point of the current column (row 0, column j).
            // In a row-major PointCloud2, the offset for a point at (row, col) is (row * width + col) * point_step.
            // For row 0, this simplifies to col * point_step.
            size_t point_start_offset = j * ros_msg->point_step;

            // Calculate the specific byte offset for the 't' field within this point's data
            size_t t_data_offset = point_start_offset + t_offset;

            // Ensure the calculated offset is within the bounds of the actual data buffer
            if (t_data_offset + sizeof(uint32_t) > ros_msg->data.size()) {
                std::cerr << "Error: Data access out of bounds for point timestamp at column " << j << ". Skipping.\n";
                break; // Exit the column loop if data is malformed or truncated
            }

            // Extract the relative timestamp value (uint32_t) from the data buffer.
            // std::memcpy is used for safe byte-level data extraction.
            uint32_t point_relative_nanosec;
            std::memcpy(&point_relative_nanosec, &ros_msg->data[t_data_offset], sizeof(uint32_t));

            // Create a rclcpp::Duration object from the relative nanoseconds
            rclcpp::Duration relative_duration(0, point_relative_nanosec);
            // Calculate the absolute timestamp for the column by adding the relative duration to the header time
            rclcpp::Time absolute_point_time = header_time + relative_duration;

            // Print the column index, its absolute timestamp in seconds, its absolute timestamp in nanoseconds,
            // and the original relative nanosecond value.
            std::cout << "\tColumn " << j << " timestamp: "
                      << std::fixed << std::setprecision(9) << absolute_point_time.seconds() << " seconds ("
                      << absolute_point_time.nanoseconds() << " ns absolute, "
                      << point_relative_nanosec << " ns relative)\n";
        }
        // --- End of new logic ---

        break; // Process one message per timer tick to avoid blocking the ROS2 executor
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::string topic_;
};

int main(int argc, char ** argv)
{
  // Check if the correct number of command-line arguments is provided
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <bag> <topic>" << std::endl;
    return 1;
  }

  // Initialize ROS2
  rclcpp::init(argc, argv);
  // Create and spin the PlaybackNode, passing the bag filename from arguments
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1], argv[2]));
  // Shutdown ROS2
  rclcpp::shutdown();

  return 0;
}
