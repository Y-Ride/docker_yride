#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ADD THESE MISSING MESSAGE TYPE HEADERS
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ADD THIS MISSING SERIALIZATION HEADER
#include <rclcpp/serialization.hpp>
// Sometimes these provide dependencies for serialization to be fully visible
// #include <rclcpp/utilities.hpp> // Added in previous iteration, keep

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <tuple>
#include <filesystem>
#include <exception> // Explicitly include for std::exception
#include <yaml-cpp/yaml.h>

// For dynamic message introspection
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
// #include <rosidl_typesupport_introspection_cpp/traits.hpp> // Still commented out
#include <rosidl_typesupport_introspection_cpp/visibility_control.h>

using namespace std::chrono_literals;

// Structure to hold topic information
struct TopicInfo
{
    std::string topic_name;
    std::string message_type;
    int message_count = 0;
    rclcpp::Time last_timestamp;
    bool received_message_since_start = false;
    std::shared_ptr<rclcpp::GenericSubscription> subscription;

    // Constructor to properly initialize all members and avoid warnings
    TopicInfo(std::string name, std::string type)
        : topic_name(std::move(name)),
          message_type(std::move(type)),
          message_count(0),
          last_timestamp(), // Default constructs to an invalid/zero timestamp
          received_message_since_start(false),
          subscription(nullptr) // Initialize shared_ptr to nullptr
    {}
};


class TopicMonitor : public rclcpp::Node
{
public:
    TopicMonitor()
        : Node("topic_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "In topic_monitor constructor");
    
        this->declare_parameter<std::string>("topics_file", "topics.yaml");
        topics_file_ = this->get_parameter("topics_file").as_string();

        load_topics_from_file();

        // Timer to stop listening and report results after 5 seconds
        RCLCPP_INFO(this->get_logger(), "Starting 5 second timer to count messaged published from topics...");
        timer_ = this->create_wall_timer(
            5s, std::bind(&TopicMonitor::report_and_shutdown, this));

        // Create an executor to process callbacks
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        // executor_->add_node(shared_from_this());

        // Spin in a separate thread to allow the main thread to continue and set up the timer
        spin_thread_ = std::thread([this]() {
            executor_->spin();
        });
    }

    ~TopicMonitor()
    {
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

    // New public method to start spinning the executor
    void start_monitoring()
    {
        executor_->add_node(shared_from_this());
    }

private:
    void load_topics_from_file()
    {
        std::string pkg_share_dir;
        try {
            pkg_share_dir = ament_index_cpp::get_package_share_directory("topic_monitor");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error getting package share directory for 'topic_monitor': %s", e.what());
            rclcpp::shutdown();
            return;
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "An unknown error occurred while getting package share directory for 'topic_monitor'.");
            rclcpp::shutdown();
            return;
        }

        std::filesystem::path file_path = pkg_share_dir;
        file_path /= topics_file_; // This will now typically be 'topics.yaml'

        RCLCPP_INFO(this->get_logger(), "Attempting to load topics from: %s", file_path.string().c_str());

        // --- YAML PARSING LOGIC ---
        try {
            YAML::Node config = YAML::LoadFile(file_path.string());

            if (config["topics"]) { // Expecting a top-level 'topics' key
                for (YAML::const_iterator it = config["topics"].begin(); it != config["topics"].end(); ++it) {
                    const YAML::Node& topic_node = *it;
                    if (topic_node["name"] && topic_node["type"]) {
                        std::string topic_name = topic_node["name"].as<std::string>();
                        std::string message_type = topic_node["type"].as<std::string>();
                        
                        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s with type: %s",
                                    topic_name.c_str(), message_type.c_str());
                        subscribe_to_topic(topic_name, message_type);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Skipping malformed topic entry in YAML (missing 'name' or 'type'): %s", YAML::Dump(topic_node).c_str());
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "YAML file '%s' does not contain a top-level 'topics' array. No topics loaded.", file_path.string().c_str());
            }
        } catch (const YAML::BadFile& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open or parse YAML file: %s. Error: %s", file_path.string().c_str(), e.what());
            rclcpp::shutdown();
            return;
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file '%s': %s", file_path.string().c_str(), e.what());
            rclcpp::shutdown();
            return;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred during YAML processing: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        // --- END YAML PARSING LOGIC ---
    }


    void subscribe_to_topic(const std::string &topic_name, const std::string &message_type)
    {
        rclcpp::QoS qos_profile(10); // Standard QoS profile

        // Use the proper constructor for TopicInfo
        auto info_it = topic_infos_.emplace(topic_name, TopicInfo(topic_name, message_type)).first;

        // Use a lambda for the generic callback
        auto callback = [this, info_it](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            info_it->second.message_count++;
            info_it->second.received_message_since_start = true;

            // Attempt to extract timestamp from the message
            rclcpp::Time stamp;
            if (extract_timestamp(msg, info_it->second.message_type, stamp))
            {
                if (!info_it->second.last_timestamp.nanoseconds()) // First message received
                {
                    info_it->second.last_timestamp = stamp;
                    if (!initial_timestamp_.nanoseconds())
                    {
                        initial_timestamp_ = stamp; // Set initial reference timestamp from the first message of any topic
                    }
                }
                else
                {
                    info_it->second.last_timestamp = stamp;
                }
            }
        };

        // Create a generic subscription
        info_it->second.subscription = this->create_generic_subscription(
            topic_name,
            message_type,
            qos_profile,
            callback);
    }

    bool extract_timestamp(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& message_type, rclcpp::Time& timestamp)
    {
        // Get the introspection type support for the message type
        // const rosidl_message_type_support_t * type_support = nullptr;
        std::string package_name = message_type.substr(0, message_type.find('/'));
        std::string msg_name = message_type.substr(message_type.find_last_of('/') + 1);

        // Workaround: We'll deserialize the message if it's one of the known types with a header.
        if (message_type == "sensor_msgs/msg/Imu") {
            sensor_msgs::msg::Imu imu_msg;
            rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
            serializer.deserialize_message(msg.get(), &imu_msg);
            timestamp = imu_msg.header.stamp;
            return true;
        }
        else if (message_type == "nav_msgs/msg/Odometry") {
            nav_msgs::msg::Odometry odom_msg;
            rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
            serializer.deserialize_message(msg.get(), &odom_msg);
            timestamp = odom_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/Image") {
            sensor_msgs::msg::Image img_msg;
            rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
            serializer.deserialize_message(msg.get(), &img_msg);
            timestamp = img_msg.header.stamp;
            return true;
        }
        // Add more message types with headers as needed
        // #   - sensor_msgs/msg/NavSatFix
        // #   - sensor_msgs/msg/PointCloud2
        else if (message_type == "sensor_msgs/msg/NavSatFix") {
            sensor_msgs::msg::NavSatFix navsatfix_msg;
            rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
            serializer.deserialize_message(msg.get(), &navsatfix_msg);
            timestamp = navsatfix_msg.header.stamp;
            return true;
        }
        else if (message_type == "sensor_msgs/msg/PointCloud2") {
            sensor_msgs::msg::PointCloud2 point_msg;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            serializer.deserialize_message(msg.get(), &point_msg);
            timestamp = point_msg.header.stamp;
            return true;
        }

        RCLCPP_WARN_ONCE(this->get_logger(), "Cannot extract timestamp for message type: %s. No specific deserializer implemented.", message_type.c_str());
        return false;
    }


    void report_and_shutdown()
    {
        timer_->cancel(); // Stop the timer

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "--------------------------------------");
        RCLCPP_INFO(this->get_logger(), "---    Topic Monitoring Results    ---");
        RCLCPP_INFO(this->get_logger(), "--------------------------------------\n");
        
        // bool all_ok = true;

        for (auto const & [topic_name, info] : topic_infos_)
        {
            if (info.message_count == 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Topic: %s (Type: %s) - ERROR: No messages published!\n",
                             topic_name.c_str(), info.message_type.c_str());
                // all_ok = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Topic: %s (Type: %s) - Messages received: %d",
                            topic_name.c_str(), info.message_type.c_str(), info.message_count);
                
                if (info.last_timestamp.nanoseconds() != 0 && initial_timestamp_.nanoseconds() != 0)
                {
                    double time_diff = std::abs((info.last_timestamp - initial_timestamp_).seconds());
                    if (time_diff > 1.0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Topic: %s - ERROR: Timestamps not synchronized! Latest: %.9f, Reference: %.9f (Diff: %.3f s)\n",
                                     topic_name.c_str(), info.last_timestamp.seconds(), initial_timestamp_.seconds(), time_diff);
                        // all_ok = false;
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Topic: %s - Timestamps synchronized (Diff: %.3f s)\n",
                                    topic_name.c_str(), time_diff);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Topic: %s - Could not check timestamp synchronization. No header found or first message not received properly.\n", topic_name.c_str());
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "-------------------------------------");
        RCLCPP_INFO(this->get_logger(), "---      Monitoring Complete      ---");
        RCLCPP_INFO(this->get_logger(), "-------------------------------------");
        rclcpp::shutdown(); // Shutdown ROS 2
    }

    std::string topics_file_;
    std::map<std::string, TopicInfo> topic_infos_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time initial_timestamp_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicMonitor>();
    // The node will manage its own spinning via the MultiThreadedExecutor in a separate thread.
    // main will just wait for ROS to shutdown, which is triggered by report_and_shutdown().
    node->start_monitoring();
    // rclcpp::spin(node); // Correct way to keep the main thread alive and let executor spin
    // After rclcpp::spin() returns (due to shutdown), the node's destructor will join spin_thread_.

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep briefly to avoid busy-waiting
    }

    rclcpp::shutdown();
    return 0;
}