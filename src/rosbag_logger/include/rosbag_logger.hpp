#ifndef _ROSBAG_LOGGER_HPP_
#define _ROSBAG_LOGGER_HPP_

#include <vector>
#include <string>
#include <thread>
#include <sstream>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <iomanip>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <sys/statvfs.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/info_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>

#include "rosbag_logger_interfaces/msg/info_message.hpp"
#include "rosbag_logger_interfaces/msg/rosbag_logger.hpp"

namespace fs = std::filesystem;
namespace cl = std::chrono_literals;

class ROSBagLogger : public rclcpp::Node
{
public:
    ROSBagLogger(); // Proper base class initialization

    ~ROSBagLogger() = default;

    struct TimeRange
    {
        int start_time;
        int end_time;
    };

    // callback functions
    void startRecordingCallback(const std_msgs::msg::Bool::SharedPtr &msg);
    void stopRecordingCallback(const std_msgs::msg::Bool::SharedPtr &msg);
    void exceptionCallback(const std_msgs::msg::Float32::SharedPtr &msg);

    // Utility functions
    void loadTopics(const std::string &file_path);
    void validateTopics(const std::string &log_file);
    bool isTopicAvailable(const std::string &topic);
    void loggerMonitoring();
    void monitorDiskSpaceAndFolder();
    void manageBagFiles();

    TimeRange getBagFileTimeRange(const std::string &bag_file_path);

    // member variables
    bool space_available = false;
    bool auto_deletion = false;

private:
    // internal methods
    void createFolderForBagFiles(const std::string &folder_path);  // fixed typo here
    std::string getTimeFormat(const std::string &date_format);
    std::string getOutputName(const std::string &dir, const std::string &prefix);
    std::string getFolderSizeInGB(const std::string &folder_path);

    bool getBagFilesContainingExceptions(const rclcpp::Time &start_time, const rclcpp::Time &end_time);
    std::pair<int, int> getStartAndEndTime(std::vector<std::tuple<rclcpp::Time, rclcpp::Time, std::string>> &bagFileList, rclcpp::Time &start_time, rclcpp::Time &end_time);
    void createExceptionBagFile(rclcpp::Time &start_time, rclcpp::Time &end_time);

    void createSessionCSV(const std::string &session_csv);
    void updateSessionInfoInCSVFile();

    // publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_start_sub;  // fixed type
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_end_sub;    // fixed type
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr add_exception_sub; // fixed type
    rclcpp::Publisher<rosbag_logger_interfaces::msg::RosbagLogger>::SharedPtr rosbag_logger_pub;

    // Recorder and topics
    std::vector<std::string> topics;

    // Exception intervals
    rclcpp::Time start_time, end_time, start_recording;
    std::list<std::pair<rclcpp::Time, rclcpp::Time>> exception_intervals_;

    // Misc. state
    bool exception_triggered_ = false;
    bool exception_triggered = false;
    bool recording_active = false;
    bool check_topics_availability = true;

    // Config values
    float exception_threshold_time{};
    int exception_start_time{};
    bool activate_exception{};
    int exception_end_time{};
    double folder_size_threshold{100};
    double disk_space_threshold{100};

    // Strings and Logging
    bool scenario_from_rosparam = false;
    bool all = false;
    bool missing_topics = false;

    std::string deleted_file_path;
    std::string topics_csv_str;
    std::string folder_path_str;
    std::string session_info_csv;
    std::string topic_list_file;
    std::string max_rosbag_size;
    std::string aios_id;

    std::string output_dir;
    std::string output_prefix;
    std::string currentFile;
    std::string filename;
    std::string directory;
    std::string outputDir;
    std::string setfile_config;
    std::string scenario_id;

    std::string time_format;
    std::string date_format;
    std::string session_start_time;
    std::string exception_start_time_;
    std::string exception_end_time_;
    std::string exception_start_end_time;
    std::string session_end_time;

    std::vector<std::string> topics_with_star;
    std::vector<std::string> topics_with_dash;
    std::vector<std::string> exception_bag_file;

    std::mutex mutex_;
    std::ofstream session_csv_file;

    rclcpp::Time exception_index;
    int number_deleted_bagfiles = 0;

    // todo: redefine this variable for ROS2
    // rosbag_logger::Rosbag_Logger rosbag_logger_msg;
    rosbag_logger_interfaces::msg::RosbagLogger rosbag_logger_msg;
};

#endif // _ROSBAG_LOGGER_HPP_
