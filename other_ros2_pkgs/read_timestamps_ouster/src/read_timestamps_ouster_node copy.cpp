#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename)
    : Node("read_timestamps_ouster")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/test/points", 10);
      timer_ = this->create_wall_timer(
          100ms, std::bind(&PlaybackNode::timer_callback, this));

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_filename;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
    }

  private:
    void timer_callback()
    {
      while (reader_->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

        if (msg->topic_name != "/ouster_top/points") {
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::PointCloud2::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        publisher_->publish(*ros_msg);
        std::cout << '(' << ros_msg->header.stamp.sec << "." << ros_msg->header.stamp.nanosec << ", frame_id: " << ros_msg->header.frame_id << ")\n";
        std::cout << '\t' << "scan " << i << 

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();

  return 0;
}








// #include <iostream>
// #include <vector>
// #include <string>
// #include <rclcpp/rclcpp.hpp>
// #include <rosbag2_cpp/reader.hpp>
// #include <rosbag2_cpp/read_options.hpp>
// #include <rosbag2_cpp/readers/sequential_reader.hpp>
// #include <rosbag2_cpp/typesupport_helpers.hpp>
// #include <rosbag2_cpp/converter_options.hpp>
// #include <rosbag2_storage/storage_options.hpp>
// #include <rosbag2_storage/topic_metadata.hpp>
// #include <rosbag2_storage/serialized_bag_message.hpp>
// #include "rosbag2_transport/reader_writer_factory.hpp"
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <rcutils/time.h>
// #include <iomanip> // For std::setfill and std::setw
// #include <cstring> // For std::memcpy

// // Function to convert a ROS2 message to a specific type
// template<typename MessageT>
// std::shared_ptr<MessageT> get_message(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message)
// {
//     auto ros_message = std::make_shared<MessageT>();
//     rclcpp::Serialization<MessageT> serialization;

//     rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

//     serialization.deserialize_message(&serialized_msg, ros_message.get());

//     return ros_message;
// }

// int main(int argc, char **argv)
// {
//     // Initialize ROS2
//     rclcpp::init(argc, argv);

//     // Check for command line arguments
//     if (argc < 2) {
//         std::cerr << "Usage: " << argv[0] << " <rosbag_path>" << std::endl;
//         return 1;
//     }

//     // Get the bag file path from the arguments
//     std::string bag_path = argv[1];
//     std::string topic_name = "/ouster_top/points";

//     // Create a ROS2 bag reader
//     rosbag2_cpp::readers::SequentialReader reader;
//     rosbag2_storage::StorageOptions storage_options{};
//     storage_options.uri = bag_path;
//     storage_options.storage_id = "sqlite3"; // Default storage plugin for ROS2 bags

//     // Converter options (required for open method)
//     rosbag2_cpp::ConverterOptions converter_options{};
//     converter_options.input_serialization_format = "cdr"; // Common serialization format
//     converter_options.output_serialization_format = "cdr";

//     // Open the bag file
//     try {
//         reader.open(storage_options, converter_options);
//     } catch (const std::exception& e) {
//         std::cerr << "Failed to open bag file: " << e.what() << std::endl;
//         return 1;
//     }

//     // Get topic metadata
//     std::vector<rosbag2_storage::TopicMetadata> topics = reader.get_all_topics_and_types();
//     bool topic_found = false;
//     for (const auto& topic : topics) {
//         if (topic.name == topic_name) {
//             topic_found = true;
//             break;
//         }
//     }

//     if (!topic_found) {
//         std::cerr << "Topic '" << topic_name << "' not found in the bag file." << std::endl;
//         return 1;
//     }

//     // Set the topic filter to read only from the specified topic
//     rosbag2_cpp::ReadOptions read_options{};
//     read_options.read_all_topics = false;
//     read_options.topics_filter = {topic_name}

//     bool first_scan_processed = false;

//     // Iterate through messages in the bag
//     // Use reader.has_next() and reader.read_next() with the read_options for filtering
//     while (reader.has_next() && !first_scan_processed) {
//         auto bag_message = reader.read_next();

//         // Check if the message is from the target topic (redundant if filter works, but good for safety)
//         if (bag_message->topic_name == topic_name) {
//             // Deserialize the PointCloud2 message
//             auto cloud_msg = get_message<sensor_msgs::msg::PointCloud2>(bag_message);

//             std::cout << "Processing first scan from topic: " << topic_name << std::endl;
//             std::cout << "Timestamp of the PointCloud2 message: "
//                       << cloud_msg->header.stamp.sec << "."
//                       << std::setfill('0') << std::setw(9) << cloud_msg->header.stamp.nanosec
//                       << std::endl;

//             // Find the 't' (timestamp) field in the PointCloud2 message
//             int t_offset = -1;
//             for (const auto& field : cloud_msg->fields) {
//                 if (field.name == "t") {
//                     t_offset = field.offset;
//                     break;
//                 }
//             }

//             if (t_offset == -1) {
//                 std::cerr << "Error: 't' (timestamp) field not found in PointCloud2 message. Skipping this message." << std::endl;
//                 continue; // Skip to the next message if 't' field is missing
//             }

//             // Iterate through each point in the first scan
//             // PointCloud2 data is typically organized as a contiguous block of bytes.
//             // Each point has a 'point_step' number of bytes.
//             // The 'row_step' is the number of bytes per row (width * point_step).
//             for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i) {
//                 // Calculate the starting byte offset for the current point
//                 size_t point_start_offset = i * cloud_msg->point_step;

//                 // Calculate the offset to the 't' field within the current point's data
//                 size_t t_data_offset = point_start_offset + t_offset;

//                 // Ensure the offset is within the bounds of the data buffer
//                 if (t_data_offset + sizeof(uint32_t) > cloud_msg->data.size()) {
//                     std::cerr << "Error: Data access out of bounds for point timestamp. Point index: " << i << std::endl;
//                     break; // Exit loop if data is malformed
//                 }

//                 // Extract the timestamp value (assuming 't' is uint32_t as per Ouster's common format)
//                 // The timestamp 't' in Ouster point clouds is typically nanoseconds since the start of the scan.
//                 // It's relative to the PointCloud2 message header timestamp.
//                 uint32_t point_relative_nanosec;
//                 std::memcpy(&point_relative_nanosec, &cloud_msg->data[t_data_offset], sizeof(uint32_t));

//                 // Convert the relative nanoseconds to an absolute timestamp
//                 // Add the relative nanoseconds to the header timestamp
//                 rclcpp::Time header_time(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
//                 rclcpp::Duration relative_duration(0, point_relative_nanosec);
//                 rclcpp::Time absolute_point_time = header_time + relative_duration;

//                 std::cout << "  Point " << i << " timestamp: "
//                           << absolute_point_time.seconds() << " seconds (relative nanosec: "
//                           << point_relative_nanosec << ")" << std::endl;
//             }
//             first_scan_processed = true; // Only process the first scan
//         }
//     }

//     if (!first_scan_processed) {
//         std::cout << "No messages found on topic '" << topic_name << "' or bag is empty." << std::endl;
//     }

//     // Shutdown ROS2
//     rclcpp::shutdown();

//     return 0;
// }
