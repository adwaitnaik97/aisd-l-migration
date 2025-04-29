#ifndef _ROSBAGS_TO_CSV_HPP_
#define _ROSBAGS_TO_CSV_HPP_

/**
 * General includes
 */
#include <sstream>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <sqlite3.h>

/**
 * Boost includes (not needed for ROS2)
 */
// #include <boost/any.hpp>
// #include <boost/foreach.hpp>

/**
 * ROS2 includes
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/qos.hpp>
#include <rcutils/time.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <pluginlib/class_loader.hpp>

/**
 * rosbag2 includes
 */
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_transport/play_options.hpp>
#include <rosbag2_transport/record_options.hpp>
// #include <rosbag2_transport/reader_writer_factory.hpp>

/**
 * QT includes
 */
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QCheckBox>

// #define foreach BOOST_FOREACH

/**
 * shape_shifter: for generic handling of the topics with unknown message types
 * ros_introspection: for introispection of the message types at compile time
 * no equivalent implementation in ROS2
 */
// #include <ros_type_introspection/ros_introspection.hpp>
// #include <topic_tools/shape_shifter.h>

struct topicInfo
{
    std::string topicName;
    std::string topicType;
    std::string serialization_format;
    std::string offered_qos_profiles;
};

class ROSBagsToCSV
{
public:
    ROSBagsToCSV(rclcpp::Node::SharedPtr node)
    {
        // Constructor implementation
        node_ = node;
    }
    ~ROSBagsToCSV()
    {
        // Destructor implementation
    }

    std::vector<std::string> topicsList;
    QString filePath;

    std::vector<topicInfo> readYAMLFile(const std::string &yamlFilePath);

    void readDB3File(const std::string &db3FilePath);

    void executeSQL(const std::string &db3FilePath);

    void createCheckBoxWidget(QDialog *dialog, const std::set<std::string> &topics);

private:
    // std::set<std::string> topics;
    std::vector<topicInfo> topics;
    std::vector<QCheckBox *> checkBoxList;
    rclcpp::Node::SharedPtr node_;

}; // class ROSBagsToCSV

#endif // _ROSBAGS_TO_CSV_HPP_