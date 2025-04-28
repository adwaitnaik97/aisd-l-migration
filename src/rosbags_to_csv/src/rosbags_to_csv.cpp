#include <MessagesHandler/messages_handler.hpp>
#include <rosbags_to_csv.hpp>
#include <sqlite3.h>

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

// void ROSBagsToCSV::executeSQL(const std::string &db3FilePath)
// {
//     sqlite3 *db;
//     char *errMsg = nullptr;

//     // Open the database
//     int rc = sqlite3_open(db3FilePath.c_str(), &db);
//     if (rc != SQLITE_OK)
//     {
//         std::cerr << "Cannot open database: " << sqlite3_errmsg(db) << std::endl;
//         sqlite3_close(db);
//         return;
//     }
//     std::cout << "Successfully opened database: " << db3FilePath << std::endl;

//     // SQL query to retrieve data from the topics table
//     const char *sql = "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics;";

//     // Callback function to process query results
//     auto callback = [](void *notUsed, int argc, char **argv, char **colNames) -> int
//     {
//         (void)notUsed; // Suppress unused variable warning
//         for (int i = 0; i < argc; i++)
//         {
//             std::cout << colNames[i] << ": " << (argv[i] ? argv[i] : "NULL") << std::endl;
//         }
//         std::cout << "----------------------------------------" << std::endl;
//         return 0;
//     };

//     // Execute the SQL query
//     rc = sqlite3_exec(db, sql, callback, nullptr, &errMsg);
//     if (rc != SQLITE_OK)
//     {
//         std::cerr << "SQL error: " << errMsg << std::endl;
//         sqlite3_free(errMsg);
//     }

//     // Close the database
//     sqlite3_close(db);
// }

void ROSBagsToCSV::readDB3File(const std::string &db3FilePath)
{
    // Create a reader for the bag file
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    try
    {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = db3FilePath;
        storage_options.storage_id = "sqlite3"; // Use SQLite3 as the storage format

        // Open the bag file
        reader->open(storage_options, rosbag2_cpp::ConverterOptions());
        std::cout << "Successfully opened bag file: " << db3FilePath << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to open bag: " << e.what() << std::endl;
        return;
    }

    // Get the list of topics
    auto topic_metadata = reader->get_all_topics_and_types();
    if (topic_metadata.empty())
    {
        std::cerr << "No topics found in the bag file." << std::endl;
        return;
    }

    // Print the list of topics
    std::cout << "Topics in the bag file:" << std::endl;
    int id = 1; // Assign a unique ID for each topic
    for (const auto &topic : topic_metadata)
    {
        std::cout << "ID: " << id << std::endl;
        std::cout << "Name: " << topic.name << std::endl;
        std::cout << "Type: " << topic.type << std::endl;
        std::cout << "Serialization Format: " << topic.serialization_format << std::endl;
        std::cout << "Offered QoS Profiles: " << topic.offered_qos_profiles << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        id++;
    }

    // Execute SQL query to print topics table
    // executeSQL(db3FilePath);
}

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("rosbags_to_csv_node");

    // Declare parameters for metadata and db3 file paths
    node->declare_parameter<std::string>("metadata_file_path", "");
    node->declare_parameter<std::string>("db3_file_path", "");

    // Get the metadata file path parameter
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

    // Get the db3 file path parameter
    std::string db3_file_path;
    if (!node->get_parameter("db3_file_path", db3_file_path) || db3_file_path.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'db3_file_path' is not set or empty. Exiting...");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Using db3 file path: %s", db3_file_path.c_str());

    // Read the db3 file
    rosbagsToCSV.readDB3File(db3_file_path);

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}