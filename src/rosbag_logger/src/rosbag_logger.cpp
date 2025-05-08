/*******************************************************
 *
 *    _    ___  ___   ___  ___ __   __ ___  ___  ___
 *   /_\  |_ _||   \ | _ \|_ _|\ \ / /| __|| _ \/ __|
 *  / _ \  | | | |) ||   / | |  \ V / | _| |   /\__ \
 * /_/ \_\|___||___/ |_|_\|___|  \_/  |___||_|_\|___/
 *
 *
 * Copyright (C) 2025 AIOS @ AIDRIVERS Ltd - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * author(s) = 'Adwait Naik'
 * email  = 'adwait@aidrivers.ai'
 *******************************************************/

#include <rosbag_logger.hpp>

ROSBagLogger::ROSBagLogger() : Node("rosbag_logger")
{
    //     auto record_start_sub = this->create_subscription<std_msgs::msg::Bool>(
    //         "/start_recording", 10,
    //         std::bind(&ROSBagLogger::startRecordingCallback, this, std::placeholders::_1));

    //     auto record_end_sub = this->create_subscription<std_msgs::msg::Bool>(
    //         "/stop_recording", 10,
    //         std::bind(&ROSBagLogger::stopRecordingCallback, this, std::placeholders::_1));

    //     auto add_exception_sub = this->create_subscription<std_msgs::msg::Float32>(
    //         "/aios/connect_triggered", 10,
    //         std::bind(&ROSBagLogger::exceptionCallback, this, std::placeholders::_1));

    //     auto rosbag_logger_pub = this->create_publisher<rosbag_logger_interfaces::msg::RosbagLogger>(
    //         "/aisd/rosbag_managment/info", 10);

    //     this->declare_parameter<std::string>("topic_list_file", "Empty");
    //     this->declare_parameter<std::string>("max_rosbag_size", "0");
    //     this->declare_parameter<std::string>("output_dir", "Empty");

    //     this->get_parameter("/aios_config/scenario_id", scenario_id);
    //     this->get_parameter("/record_sensors/output_prefix", output_prefix);
    //     this->get_parameter("/rosbag_config/activate_exception", activate_exception);
    //     this->get_parameter("/rosbag_config/exception_threshold_time", exception_threshold_time);
    //     this->get_parameter("/rosbag_config/exception_start_time", exception_start_time);
    //     this->get_parameter("/rosbag_config/exception_end_time", exception_end_time);
    //     this->get_parameter("/rosbag_config/disk_space_threshold", disk_space_threshold);
    //     this->get_parameter("/rosbag_config/folder_size_threshold", folder_size_threshold);
    //     this->get_parameter("/rosbag_logger/auto_deletion", auto_deletion);

    //     loadTopics(topic_list_file);

    //     time_format = getTimeFormat("Time");

    //     date_format = getTimeFormat("Date");

    //     session_start_time = "";

    //     session_end_time = "";

    //     exception_start_time_ = "";

    //     folder_path_str = output_dir + "/" + date_format;

    //     topics_csv_str = output_dir + "/" + date_format + "/" + scenario_id + "_" + time_format + "_logger.csv";

    //     session_info_csv = output_dir + "/" + date_format + "/" + scenario_id + "_" + time_format + "_session_info.csv";

    //     deleted_file_path = output_dir + "/" + date_format + "/" + scenario_id + "_" + time_format + "_deleted_bags.csv";

    //     start_time = rclcpp::Time(0);

    //     end_time = rclcpp::Time(0);

    //     start_recording = rclcpp::Time(0);

    //     exception_index = rclcpp::Time(0);

    //     createFolderForBagFiles(folder_path_str);

    //     createSessionCSV(session_info_csv);

    //     if (activate_exception)
    //     {
    //         std::thread copy_thread([this]()
    //                                 { this->manageBagFiles(); });

    //         copy_thread.detach();
    //     }
}

void ROSBagLogger::createFolderForBagFiles(const std::string &folder_path)
{
    fs::path directory_path(folder_path);

    if (!fs::exists(directory_path))
    {
        fs::create_directories(folder_path);
        RCLCPP_INFO(this->get_logger(), "Created directory: %s", folder_path.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Directory already exists: %s", folder_path.c_str());
    }
}

void ROSBagLogger::createSessionCSV(const std::string &session_csv)
{
    session_csv_file.open(session_csv, std::ios::app);
    if (!session_csv_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open session CSV file: %s", session_csv.c_str());
        return;
    }

    session_csv_file << "Session ID,Start Time,End Time,Exception Start Time,Exception End Time\n";
    session_csv_file.close();
}

// Todo:: This function needs to be implemented differently
// Observation: In ROS2, there is no file generated with ".active" extension to indicate the data is being recorded currently.
// In ROS2, the metadata.yaml file is created once the recording is stopped.
void ROSBagLogger::updateSessionInfoInCSVFile()
{
    session_csv_file << session_start_time << ",";
    session_csv_file << session_end_time << ",";

    int N_bagfiles = 0;
    int E_bagfiles = 0;
    int T_bagfiles = 0;

    for (const auto &entry : fs::directory_iterator(output_dir))
    {
        if (!entry.is_directory())
            continue;

        std::string dir_path = entry.path().string();
        std::string dir_name = entry.path().filename().string();
        std::string active_bagfile;
        bool has_db3 = false;
        bool has_metadata = false;
        bool is_exception = dir_name.find("exception") != std::string::npos;

        for (const auto &file : fs::directory_iterator(entry.path()))
        {
            if (file.path().extension() == ".db3")
                has_db3 = true;
            else if (file.path().filename() == "metadata.yaml")
                has_metadata = true;
        }

        if (is_exception)
        {
            E_bagfiles++;
        }
        else if (has_db3 && has_metadata)
        {
            N_bagfiles++; // Completed bag
        }
        else if (has_db3 && !has_metadata)
        {
            active_bagfile = dir_name; // In-progress bag
        }
    }

    T_bagfiles = N_bagfiles + E_bagfiles;
    session_csv_file << N_bagfiles << "," << E_bagfiles << "," << T_bagfiles << ",";

    session_csv_file << session_end_time;
    session_csv_file.flush();
}

std::string ROSBagLogger::getTimeFormat(const std::string &date_format)
{
    rclcpp::Clock clock;
    rclcpp::Time stamp = clock.now();

    if (stamp.nanoseconds() == 0)
    {
        std::time_t t = std::time(nullptr);
        std::tm *tm_ptr = std::localtime(&t);

        std::stringstream ss;
        if (date_format == "Time")
            ss << std::put_time(tm_ptr, "%Y-%m-%d_%H-%M-%S");
        else if (date_format == "Date")
            ss << std::put_time(tm_ptr, "%Y-%m-%d");
        return ss.str();
    }

    std::time_t t = stamp.seconds(); // or stamp.nanoseconds() / 1e9
    std::tm *tm_ptr = std::localtime(&t);

    std::stringstream ss;
    if (date_format == "Time")
        ss << std::put_time(tm_ptr, "%Y-%m-%d_%H-%M-%S");
    else if (date_format == "Date")
        ss << std::put_time(tm_ptr, "%Y-%m-%d");
    return ss.str();
}

void ROSBagLogger::exceptionCallback(const std_msgs::msg::Float32::SharedPtr &msg)
{
    rclcpp::Clock clock;
    rclcpp::Time now = clock.now();

    if (msg->data == 1.0f && !exception_triggered_)
    {
        exception_triggered_ = true;
        start_time = now - rclcpp::Duration::from_seconds(exception_threshold_time);

        if ((start_time - start_recording).seconds() <= exception_threshold_time)
        {
            start_time = now;
        }

        exception_start_time_ = "(" + getTimeFormat("Time") + " : ";
        RCLCPP_INFO(this->get_logger(), "Start time is: %s", exception_start_time_.c_str());
    }
    else if (msg->data == 0.0f && exception_triggered_)
    {
        exception_triggered_ = false;
        end_time = now + rclcpp::Duration::from_seconds(exception_threshold_time);
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO(this->get_logger(), "End time is: %s", exception_start_time_.c_str());

        if (start_time < end_time && start_time != rclcpp::Time(0))
        {
            exception_end_time_ = getTimeFormat("Time") + ") ";
            exception_start_end_time = exception_start_time_ + exception_end_time_;
            exception_intervals_.push_back({start_time, end_time});
        }

        start_time = rclcpp::Time(0);
    }
}

std::pair<int, int> ROSBagLogger::getStartAndEndTime(std::vector<std::tuple<rclcpp::Time, rclcpp::Time, std::string>> &bagFileList, rclcpp::Time &start_time, rclcpp::Time &end_time)
{
    size_t currentBagFileIndex{};
    for (auto &[bag_start_time, bag_end_time, bag_file] : bagFileList)
    {
        if (start_time >= bag_start_time && start_time <= bag_end_time)
        {
            size_t lastBagFileIndex = currentBagFileIndex;
            for (size_t idx = currentBagFileIndex; idx < bagFileList.size(); idx++)
            {
                auto &[next_bag_start_time, next_bag_end_time, next_bag_file] = bagFileList[idx];
                if (end_time >= next_bag_start_time && end_time <= next_bag_end_time)
                {
                    lastBagFileIndex = idx;
                    return {currentBagFileIndex, lastBagFileIndex};
                }
            }
        }

        currentBagFileIndex++;
    }

    return {-1, -1};
}

void ROSBagLogger::loggerMonitoring()
{
    validateTopics(topics_csv_str);

    for (const auto &entry : fs::directory_iterator(output_dir))
    {
        std::string dir_path = entry.path().string();
        std::string dir_name = entry.path().filename().string();

        bool has_db3 = false;
        bool has_metadata = false;

        for (const auto &file : fs::directory_iterator(entry.path()))
        {
            if (file.path().extension() == ".db3")
            {
                has_db3 = true;
            }
            else if (file.path().filename() == "metadata.yaml")
            {
                has_metadata = true;
            }
        }

        if (has_db3 && has_metadata)
        {
            // auto rosbag_logger_msg = rosbag_logger_interfaces::msg::RosbagLogger();
            rosbag_logger_msg.active_bagfile = dir_name;
            rosbag_logger_msg.health = 1;

            rosbag_logger_pub->publish(rosbag_logger_msg);
            // Optionally log or break if you only want to report one
        }
    }
}

void ROSBagLogger::loadTopics(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open topics file : %s", file_path.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line.find('#') != std::string::npos)
        {
            continue;
        }
        topics.push_back(line);
    }
    file.close();
}

bool ROSBagLogger::isTopicAvailable(const std::string &topic)
{
    auto topics = this->get_topic_names_and_types();
    return topics.find(topic) != topics.end();
}

void ROSBagLogger::validateTopics(const std::string &log_file)
{
    if (check_topics_availability)
    {
        rosbag_logger_msg.info.inactive_topics = 0;
        rosbag_logger_msg.info.active_topics = 0;
        std::ofstream csv_file(log_file, std::ios::out | std::ios::app);
        if (!csv_file)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_file.c_str());
            return;
        }

        for (const auto &topic : topics)
        {
            if (!isTopicAvailable(topic))
            {
                rosbag_logger_msg.info.inactive_topics += 1;
                csv_file << topic << ", Not Available" << std::endl;
            }
            else
            {
                rosbag_logger_msg.info.active_topics += 1;
            }
        }
        csv_file.close();
    }
    check_topics_availability = false;
}

void ROSBagLogger::monitorDiskSpaceAndFolder()
{
    struct statvfs stat;
    if (statvfs("/", &stat) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get disk space statistics");
        return;
    }

    uinit64_t free_space = stat.f_bavail * stat.f_frsize;
    if (free_space < disk_space_threshold)
    {
        RCLCPP_WARN(this->get_logger(), "Low disk space: %lu bytes available", free_space);
    }

    double folder_size = std::stod(getFolderSizeInGB(output_dir));

    rosbag_logger_msg.info.file_size = folder_size;

    if (folder_size >= folder_size_threshold)
    {
        space_available = false;
    }
    else
    {
        space_available = true
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROSBagLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}