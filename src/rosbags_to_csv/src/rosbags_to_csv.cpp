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
 * author(s) = 'Mahmoud Alsayed','Adwait Naik'
 * email  = 'Mahmoud@aidrivers.ai', 'adwait@aidrivers.ai'
 *******************************************************/

#include <MessagesHandler/messages_handler.hpp>
#include <rosbags_to_csv.hpp>

std::vector<topicInfo> ROSBagsToCSV::readYAMLFile(const std::string &yamlFilePath)
{
    std::vector<topicInfo> topic_info;

    try
    {
        YAML::Node config = YAML::LoadFile(yamlFilePath);
        if (config["rosbag2_bagfile_information"])
        {
            auto topics_node = config["rosbag2_bagfile_information"]["topics_with_message_count"];

            if (topics_node && topics_node.IsSequence())
            {
                for (const auto &topic_entry : topics_node)
                {
                    if (topic_entry["topic_metadata"])
                    {
                        topicInfo info;
                        info.topicName = topic_entry["topic_metadata"]["name"].as<std::string>();
                        info.topicType = topic_entry["topic_metadata"]["type"].as<std::string>();
                        topic_info.push_back(info);
                    }
                }

                std::cout << "Topics in the YAML file:" << std::endl;
                for (const auto &topic : topic_info)
                {
                    std::cout << "Topic Name: " << topic.topicName << ", Topic Type: " << topic.topicType << std::endl;
                }
            }
            else
            {
                std::cerr << "No valid topics found in the YAML file." << std::endl;
            }
        }
        else
        {
            std::cerr << "Invalid YAML structure: 'rosbag2_bagfile_information' not found." << std::endl;
        }
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }

    return topic_info;
}

void ROSBagsToCSV::readDB3File(const std::string &db3FilePath)
{
    // Create a reader for the bag file
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    try
    {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = db3FilePath;
        storage_options.storage_id = "sqlite3"; // Use SQLite3 as the storage format
        reader->open(storage_options, rosbag2_cpp::ConverterOptions());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to open bag: " << e.what() << std::endl;
        return;
    }

    // Get the list of topics
    auto topic_metadata = reader->get_all_topics_and_types();
    for (const auto &topic : topic_metadata)
    {
        topicsList.push_back(topic.name);
    }
}

// void ROSBagsToCSV::getTopicList(const std::string &path)
// {
//     // Create a reader for the bag file
//     std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
//     try
//     {
//         rosbag2_cpp::StorageOptions storage_options;
//         storage_options.uri = path;
//         storage_options.storage_id = "sqlite3"; // Use SQLite3 as the storage format
//         reader->open(storage_options, rosbag2_cpp::ConverterOptions());
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "Failed to open bag: " << e.what() << std::endl;
//         return;
//     }

//     // Get the list of topics
//     std::set<std::string> topics;

//     auto topic_metadata = reader->get_all_topics_and_types();
//     for (const auto &topic : topic_metadata)
//     {
//         topics.insert(topic.name);
//     }

//     // Print the list of topics
//     std::cout << "Topics in the bag file:" << std::endl;
//     for (const auto &topic : topics)
//     {
//         std::cout << topic << std::endl;
//     }

//     // Create a dialog to display the topics
//     QDialog *dialog = new QDialog();
//     createCheckBoxWidget(dialog, topics);

//     std::cout << "hey " << topicsList.size() << std::endl;
//     dialog->exec();
// }

// void ROSBagsToCSV::createCheckBoxWidget(QDialog *dialog, const std::set<std::string> &topics)
// {
//     QVBoxLayout *layout = new QVBoxLayout(dialog);
//     QCheckBox *checkBox;
//     for (const auto &topic : topics)
//     {
//         checkBox = new QCheckBox(QString::fromStdString(topic));
//         checkBox->setChecked(true);
//         layout->addWidget(checkBox);
//         checkBoxList.push_back(checkBox);
//     }

//     QScrollArea *scrollArea = new QScrollArea(dialog);
//     QWidget *scrollWidget = new QWidget();
//     scrollWidget->setLayout(layout);

//     scrollArea->setWidget(scrollWidget);
//     scrollArea->setWidgetResizable(true);
//     dialog->setLayout(new QVBoxLayout());
//     dialog->layout()->addWidget(scrollArea);
//     QPushButton *captureButton = new QPushButton("select");
//     dialog->layout()->addWidget(captureButton);

//     QObject::connect(captureButton, &QPushButton::clicked, [&](){
//         qDebug() << "Selected checkboxes: ";
//         for (QCheckBox* Box : checkBoxList) {
//             if (Box->isChecked()) {
//                 // Todo:: Need to find an alternative for RosIntrospection::Parser parser;
//                 topicsList.push_back((Box->text().toStdString()));

//                 std::string csv_path = filePath.toStdString() + std::string(".csv");

//                 std::ofstream file(csv_path, std::ios_base::app);
//                 if (!file.is_open()) {
//                     std::cerr << "Failed to open file: " << csv_path << std::endl;
//                     return;
//                 }

//             }

//         } });

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("rosbags_to_csv_node");

    // Declare and get the metadata file path parameter
    node->declare_parameter<std::string>("metadata_file_path", "/home/adwait/workspace/ros2_packages/aisd-l-migration/src/rosbags_to_csv/bag_files/rosbag2_2025_04_24-10_45_07/metadata.yaml");
    std::string metadata_file_path;
    if (!node->get_parameter("metadata_file_path", metadata_file_path) || metadata_file_path.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'metadata_file_path' is not set or empty. Exiting...");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Using metadata file path: %s", metadata_file_path.c_str());

    // Create an instance of ROSBagsToCSV and process the YAML file
    ROSBagsToCSV rosbagsToCSV(node);
    auto topics = rosbagsToCSV.readYAMLFile(metadata_file_path);

    if (topics.empty())
    {
        RCLCPP_WARN(node->get_logger(), "No topics found in the metadata file.");
    }

    node->declare_parameter<std::string>("db3_file_path", "/home/adwait/workspace/ros2_packages/aisd-l-migration/src/rosbags_to_csv/bag_files/rosbag2_2025_04_24-10_45_07/rosbag2_2025_04_24-10_45_07_0.db3");

    std::string db3_file_path;
    if (!node->get_parameter("db3_file_path", db3_file_path) || db3_file_path.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'db3_file_path' is not set or empty. Exiting...");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Using db3 file path: %s", db3_file_path.c_str());

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    // rclcpp::shutdown();
    return 0;
}
