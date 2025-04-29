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

                        // Extract serialization_format
                        if (topic_entry["topic_metadata"]["serialization_format"])
                        {
                            info.serialization_format = topic_entry["topic_metadata"]["serialization_format"].as<std::string>();
                        }
                        else
                        {
                            info.serialization_format = "N/A";
                        }

                        // Extract offered_qos_profiles
                        if (topic_entry["topic_metadata"]["offered_qos_profiles"])
                        {
                            info.offered_qos_profiles = topic_entry["topic_metadata"]["offered_qos_profiles"].as<std::string>();
                        }
                        else
                        {
                            info.offered_qos_profiles = "N/A";
                        }

                        topic_info.push_back(info);
                    }
                }

                // Print the topics and their metadata
                std::cout << "Topics in the YAML file:" << std::endl;
                for (const auto &topic : topic_info)
                {
                    std::cout << "Topic Name: " << topic.topicName << std::endl;
                    std::cout << "Topic Type: " << topic.topicType << std::endl;
                    std::cout << "Serialization Format: " << topic.serialization_format << std::endl;
                    std::cout << "Offered QoS Profiles: " << topic.offered_qos_profiles << std::endl;
                    std::cout << "----------------------------------------" << std::endl;
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

void ROSBagsToCSV::executeSQL(const std::string &db3FilePath)
{
    sqlite3 *db;
    char *errMsg = nullptr;

    // Open the database
    int rc = sqlite3_open(db3FilePath.c_str(), &db);
    if (rc != SQLITE_OK)
    {
        std::cerr << "Cannot open database: " << sqlite3_errmsg(db) << std::endl;
        sqlite3_close(db);
        return;
    }
    std::cout << "Successfully opened database: " << db3FilePath << std::endl;

    // SQL query to retrieve data from the topics table
    const char *sql = "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics;";

    // Callback function to process query results
    auto callback = [](void *notUsed, int argc, char **argv, char **colNames) -> int
    {
        (void)notUsed; // Suppress unused variable warning
        for (int i = 0; i < argc; i++)
        {
            std::cout << colNames[i] << ": " << (argv[i] ? argv[i] : "NULL") << std::endl;
        }
        std::cout << "----------------------------------------" << std::endl;
        return 0;
    };

    // Execute the SQL query
    rc = sqlite3_exec(db, sql, callback, nullptr, &errMsg);
    if (rc != SQLITE_OK)
    {
        std::cerr << "SQL error: " << errMsg << std::endl;
        sqlite3_free(errMsg);
    }

    // Close the database
    sqlite3_close(db);
}

// void ROSBagsToCSV::readDB3File(const std::string &db3FilePath)
// {
//     // Create a reader instance
//     std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();

//     try
//     {
//         rosbag2_storage::StorageOptions storage_options;
//         storage_options.uri = db3FilePath;
//         storage_options.storage_id = "sqlite3"; // Use SQLite3 as the storage format

//         // Open the bag file
//         reader->open(storage_options, rosbag2_cpp::ConverterOptions());
//         std::cout << "Successfully opened bag file: " << db3FilePath << std::endl;
//     }

//     catch (const std::exception &e)
//     {
//         std::cerr << "Failed to open bag: " << e.what() << std::endl;
//         return;
//     }

//     // Get the list of topics
//     auto topic_metadata = reader->get_all_topics_and_types();
//     if (topic_metadata.empty())
//     {
//         std::cerr << "No topics found in the bag file." << std::endl;
//         return;
//     }
//     // Print the list of topics
//     std::cout << "Topics in the bag file:" << std::endl;
//     int id = 1; // Assign a unique ID for each topic
//     for (const auto &topic : topic_metadata)
//     {
//         std::cout << "ID: " << id << std::endl;
//         std::cout << "Name: " << topic.name << std::endl;
//         std::cout << "Type: " << topic.type << std::endl;
//         std::cout << "Serialization Format: " << topic.serialization_format << std::endl;
//         std::cout << "Offered QoS Profiles: " << topic.offered_qos_profiles << std::endl;
//         std::cout << "----------------------------------------" << std::endl;
//         id++;
//     }

//     // Execute SQL query to print topics table
//     // executeSQL(db3FilePath);
// }

// void ROSBagsToCSV::createCheckBoxWidget(QDialog *dialog, const std::set<std::string> &topics)
// {
//     QVBoxLayout *layout = new QVBoxLayout(dialog);
//     QCheckBox *checkBox;
//     for (const auto &topic : topics)
//     {
//         checkBox = new QCheckBox(QString::fromStdString(topic));
//         checkBoxList.push_back(checkBox);
//         layout->addWidget(checkBox);
//     }

//     QScrollArea *scrollArea = new QScrollArea(dialog);
//     QWidget *scrollWidget = new QWidget(scrollArea);
//     scrollWidget->setLayout(layout);
//     scrollArea->setWidget(scrollWidget);
//     scrollArea->setWidgetResizable(true);

//     dialog->setLayout(new QVBoxLayout(dialog));
//     dialog->layout()->addWidget(scrollArea);
//     QPushButton *captureButton = new QPushButton("select");
//     dialog->layout()->addWidget(captureButton);

//     QObject::connect(captureButton, &QPushButton::clicked, [&]()
//                      {
//                          qDebug() << "Selected checkboxes:";
//                          for (QCheckBox *Box : checkBoxList)
//                          {
//                              if (Box->isChecked())
//                              {
//                                  topicsList.push_back((Box->text()).toStdString());
//                                  std::string path_name = filePath.toStdString() + std::string(".csv");

//                                  // Configure storage options
//                                  rosbag2_storage::StorageOptions storage_options;
//                                  storage_options.uri = filePath.toStdString();
//                                  storage_options.storage_id = "sqlite3"; // Use SQLite3 as the storage format

//                                  // Configure converter options
//                                  rosbag2_cpp::ConverterOptions converter_options;
//                                  converter_options.input_serialization_format = "cdr";
//                                  converter_options.output_serialization_format = "cdr";

//                                  std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();

//                                  try
//                                  {
//                                      reader->open(storage_options, rosbag2_cpp::ConverterOptions());
//                                      std::cout << "Successfully opened bag file: " << filePath.toStdString() << std::endl;

//                                     //  // Iterate through messages in the bag file
//                                     //  while (reader->has_next())
//                                     //  {
//                                     //      auto bag_message = reader->read_next();
//                                     //      const std::string &topic_name = bag_message->topic_name;

//                                     //      // Filter messages by topics in topicsList
//                                     //      if (std::find(topicsList.begin(), topicsList.end(), topic_name) != topicsList.end())
//                                     //      {
//                                     //          std::cout << "Topic: " << topic_name << std::endl;
//                                     //          std::cout << "Message: " << bag_message->serialized_data->buffer_length << " bytes" << std::endl;
//                                     //      }
//                                     //  }

//                                     // Get all topics and their metadata
//                                     auto topics_metadata = reader->get_all_topics_and_types();

//                                     // Iterate through the topics and register message definitions
//                                     for (const auto &topic_metadata : topics_metadata)
//                                     {
//                                         const std::string &topic_name = topic_metadata.name;
//                                         const std::string &datatype = topic_metadata.type;
//                                         const std::string &serialization_format = topic_metadata.serialization_format;

//                                         // Register the type using the topic_name as an identifier
//                                         std::string definition = ""; // ROS 2 does not provide message definitions in the bag file
//                                         //parser.registerMessageDefinition(topic_name, RosIntrospection::ROSType(datatype), definition);

//                                         std::cout << "Registered topic: " << topic_name << std::endl;
//                                         std::cout << "  Datatype: " << datatype << std::endl;
//                                         std::cout << "  Serialization Format: " << serialization_format << std::endl;
//                                     }
//                                  }
//                                  catch (const std::exception &e)
//                                  {
//                                     std::cerr << "Error reading bag file: " << e.what() << std::endl;
//                                  }

//                                 // todo:: RosIntrospection::Parser parser;
//                             //     for (const rosbag::ConnectionInfo* connection : bag_view.getConnections()) {
//                             //         const std::string& topic_name = connection->topic;
//                             //         const std::string& datatype = connection->datatype;
//                             //         const std::string& definition = connection->msg_def;
//                             //         // register the type using the topic_name as identifier.
//                             //         parser.registerMessageDefinition(topic_name, RosIntrospection::ROSType(datatype),
//                             //                                          definition);
//                             //       }
//                             //       std::map<std::string, RosIntrospection::FlatMessage> flat_containers;
//                             //       std::map<std::string, RosIntrospection::RenamedValues> renamed_vectors;
//                             //       std::vector<uint8_t> buffer;
//                             //       for (rosbag::MessageInstance msg_instance : bag_view) {
//                             //         const std::string& topic_name = msg_instance.getTopic();
//                             //         const size_t msg_size = msg_instance.size();
//                             //         buffer.resize(msg_size);
//                             //         ros::serialization::OStream stream(buffer.data(), buffer.size());
//                             //         msg_instance.write(stream);

//                             //         RosIntrospection::FlatMessage& flat_container = flat_containers[topic_name];
//                             //         RosIntrospection::RenamedValues& renamed_values = renamed_vectors[topic_name];
//                             //         parser.deserializeIntoFlatContainer(topic_name, RosIntrospection::Span<uint8_t>(buffer),
//                             //                                             &flat_container, 100);

//                             //         parser.applyNameTransform(topic_name, flat_container, &renamed_values);
//                             //         printf("--------- %s ----------\n", topic_name.c_str());
//                             //         std::string bool_value;
//                             //         for (auto it : renamed_values) {
//                             //           const std::string& key = it.first;
//                             //           const RosIntrospection::Variant& value = it.second;
//                             //           // printf(" %s = %f\n", key.c_str(), value.convert<std::string>() );
//                             //           file << key.c_str() << ": ";
//                             //           if(value.convert<double>() == 0){
//                             //             bool_value = "False";
//                             //           }
//                             //           else if(value.convert<double>() == 1){
//                             //              bool_value = "True";
//                             //           }
//                             //           file << bool_value<< ", ";
//                             //         }
//                             //          file <<"\n";
//                             //         for (auto it : flat_container.name) {
//                             //           const std::string& key = it.first.toStdString();
//                             //           const std::string& value = it.second;
//                             //           printf(" %s = %s\n", key.c_str(), value.c_str());
//                             //           if(!value.empty()){
//                             //             file << value.c_str() << "\n";
//                             //           }
//                             //         }
//                             //       }
//                             //       file.close();
//                             //     }
//                             //   }
//                             //   dialog->close();
//                                 // Check if the csv file opened successfully
//                                 std::ofstream csvFile(path_name);
//                                 if (!csvFile.is_open())
//                                 {
//                                     std::cerr << "Error opening CSV file: " << path_name << std::endl;
//                                     return;
//                                 }
//                              }
//                          } });
// }

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("rosbags_to_csv_node");

    // Declare parameters for metadata and db3 file paths
    node->declare_parameter<std::string>("metadata_file_path", "");
    node->declare_parameter<std::string>("db3_file_path", "");

    // // Get the metadata file path parameter
    std::string metadata_file_path;
    if (!node->get_parameter("metadata_file_path", metadata_file_path) || metadata_file_path.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'metadata_file_path' is not set or empty. Exiting...");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Using metadata file path: %s", metadata_file_path.c_str());

    // // Create an instance of ROSBagsToCSV and process the YAML file
    ROSBagsToCSV rosbagsToCSV(node);
    auto topics = rosbagsToCSV.readYAMLFile(metadata_file_path);

    if (topics.empty())
    {
        RCLCPP_WARN(node->get_logger(), "No topics found in the metadata file.");
    }

    // Get the db3 file path parameter
    std::string db3_file_path;
    if (!node->get_parameter("db3_file_path", db3_file_path) || db3_file_path.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'db3_file_path' is not set or empty. Exiting...");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Using db3 file path: %s", db3_file_path.c_str());

    // Read the db3 file
    // rosbagsToCSV.readDB3File(db3_file_path);
    rosbagsToCSV.executeSQL(db3_file_path);

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}