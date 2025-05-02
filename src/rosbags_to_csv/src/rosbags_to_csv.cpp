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

    // Helper function to execute SQL queries and write results to CSV
    auto executeAndWriteToCSV = [&](const char *query, const std::string &tableName)
    {
        std::cout << "Querying table: " << tableName << std::endl;

        // Data container for rows
        std::vector<std::vector<std::string>> data;
        std::vector<std::string> headers;

        // Callback function to collect query results
        auto callback = [](void *dataPtr, int argc, char **argv, char **colNames) -> int
        {
            auto *data = static_cast<std::vector<std::vector<std::string>> *>(dataPtr);

            // Add column names as headers if not already added
            if (data->empty())
            {
                std::vector<std::string> headers;
                for (int i = 0; i < argc; ++i)
                {
                    headers.push_back(colNames[i]);
                }
                data->push_back(headers);
            }

            // Add row data
            std::vector<std::string> row;
            for (int i = 0; i < argc; ++i)
            {
                row.push_back(argv[i] ? argv[i] : "NULL");
            }
            data->push_back(row);

            return 0;
        };

        // Execute the query
        int rc = sqlite3_exec(db, query, callback, &data, &errMsg);
        if (rc != SQLITE_OK)
        {
            std::cerr << "SQL error in table " << tableName << ": " << errMsg << std::endl;
            sqlite3_free(errMsg);
            return;
        }

        // Write data to CSV
        std::string csvDirectory = "/home/adwait/workspace/ros2_packages/aisd-l-migration/src/rosbags_to_csv/config/csv_files/";
        std::string csvFilePath = csvDirectory + tableName + ".csv";
        std::ofstream csvFile(csvFilePath);
        if (!csvFile.is_open())
        {
            std::cerr << "Error: Could not open CSV file for writing: " << csvFilePath << std::endl;
            return;
        }

        // Write headers
        if (!data.empty())
        {
            headers = data.front();
            for (size_t i = 0; i < headers.size(); ++i)
            {
                csvFile << headers[i];
                if (i < headers.size() - 1)
                {
                    csvFile << ",";
                }
            }
            csvFile << "\n";

            // Write rows
            for (size_t i = 1; i < data.size(); ++i)
            {
                const auto &row = data[i];
                for (size_t j = 0; j < row.size(); ++j)
                {
                    csvFile << row[j];
                    if (j < row.size() - 1)
                    {
                        csvFile << ",";
                    }
                }
                csvFile << "\n";
            }
        }

        csvFile.close();
        std::cout << "Data successfully written to CSV file: " << csvFilePath << std::endl;
    };

    // Execute queries and write results to CSV files
    executeAndWriteToCSV("SELECT * FROM 'message_definitions' LIMIT 0,30", "message_definitions");
    executeAndWriteToCSV("SELECT * FROM 'messages' LIMIT 0,30", "messages");
    executeAndWriteToCSV("SELECT * FROM 'metadata' LIMIT 0,30", "metadata");
    executeAndWriteToCSV("SELECT * FROM 'schema' LIMIT 0,30", "schema");
    executeAndWriteToCSV("SELECT * FROM 'topics' LIMIT 0,30", "topics");

    // Close the database
    sqlite3_close(db);
}

void ROSBagsToCSV::viewTopicInfo(QDialog *dialog, const std::vector<topicInfo> &topics)
{
    QVBoxLayout *layout = new QVBoxLayout(dialog);

    QLabel *instruction = new QLabel("Select topics to view details:", dialog);
    layout->addWidget(instruction);

    for (const auto &info : topics)
    {
        // Create checkbox
        QCheckBox *checkbox = new QCheckBox(QString::fromStdString(info.topicName), dialog);
        layout->addWidget(checkbox);

        // Create a group box to show details (initially hidden)
        QGroupBox *detailsBox = new QGroupBox("Details", dialog);
        QVBoxLayout *detailsLayout = new QVBoxLayout(detailsBox);

        detailsLayout->addWidget(new QLabel("Type: " + QString::fromStdString(info.topicType)));
        detailsLayout->addWidget(new QLabel("Serialization Format: " + QString::fromStdString(info.serialization_format)));
        detailsLayout->addWidget(new QLabel("QoS Profiles: " + QString::fromStdString(info.offered_qos_profiles)));

        detailsBox->setVisible(false);
        layout->addWidget(detailsBox);

        // Connect checkbox toggle to show/hide detail box
        QObject::connect(checkbox, &QCheckBox::toggled, detailsBox, &QGroupBox::setVisible);
    }

    QPushButton *closeButton = new QPushButton("Close", dialog);
    QObject::connect(closeButton, &QPushButton::clicked, dialog, &QDialog::accept);
    layout->addWidget(closeButton);

    QPixmap logoPixmap("/home/adwait/workspace/ros2_packages/aisd-l-migration/src/rosbags_to_csv/config/logo/aidriversltd_logo.jpg"); // Update with your actual path
    QLabel *logoLabel = new QLabel(dialog);
    logoLabel->setPixmap(logoPixmap.scaled(150, 150, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    logoLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(logoLabel);

    // === Add Instructions ===
    dialog->setLayout(layout);
    dialog->setWindowTitle("Topic Information Viewer");
    dialog->resize(600, 500);
    dialog->exec();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rosbags_to_csv_node");

    node->declare_parameter<std::string>("metadata_file_path", "");
    node->declare_parameter<std::string>("db3_file_path", "");

    std::string metadata_file_path;
    node->get_parameter("metadata_file_path", metadata_file_path);

    std::string db3_file_path;
    node->get_parameter("db3_file_path", db3_file_path);

    ROSBagsToCSV rosbagsToCSV(node);
    auto topics = rosbagsToCSV.readYAMLFile(metadata_file_path);
    rosbagsToCSV.executeSQL(db3_file_path);

    // Spin ROS 2 in a separate thread
    std::thread spin_thread([&]()
                            { rclcpp::spin(node); });

    QApplication app(argc, argv);
    QDialog dialog;
    rosbagsToCSV.viewTopicInfo(&dialog, topics);

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
