#include <cstdio>
#include <memory>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "JLB/udp.hxx"
#include "JLB/common.hxx"
#include "JLB/signals.hxx"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace jlb
{

    class Telemetry : public rclcpp::Node
    {
    public:
        Telemetry() : Node("telemetry"), server(SERVER_ADDRESS, SERVER_PORT), tf_broadcaster(this)
        {
            RCLCPP_INFO(get_logger(), "node started.");

            parse_map("competition.map");

            map_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
            pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
            map_timer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Telemetry::map_timer_callback, this));
            udp_timer = create_wall_timer(std::chrono::milliseconds(10), std::bind(&Telemetry::upd_timer_callback, this));

            for (const auto &signal : jlb::SignalLibrary::get_instance().signals)
            {
                publishers.push_back(create_publisher<std_msgs::msg::Float32>(signal.name, 10));
            }
        }
        ~Telemetry()
        {
            RCLCPP_INFO(get_logger(), "node stopped.");
        }

    private:
        std::vector<int8_t> map;

        UDPServer server;
        std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers;

        nav_msgs::msg::OccupancyGrid map_msg;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
        tf2_ros::StaticTransformBroadcaster tf_broadcaster;

        geometry_msgs::msg::PoseStamped pose_msg;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;

        rclcpp::TimerBase::SharedPtr map_timer;
        rclcpp::TimerBase::SharedPtr udp_timer;

        void parse_map(const std::string &filename)
        {
            std::ifstream map_file;
            auto robonaut_telemetry_share_dir = ament_index_cpp::get_package_share_directory("robonaut-telemetry");
            auto file_path = robonaut_telemetry_share_dir + "/maps/" + filename;

            map_file.open(file_path);
            std::string line;
            std::getline(map_file, line);
            if (map_file.is_open())
            {
                RCLCPP_INFO(this->get_logger(), "map file opened.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "map file not found.");
                return;
            }
            auto map_width = std::stoul(line);
            std::getline(map_file, line);
            auto map_height = std::stoul(line);

            std::getline(map_file, line);

            // populate data
            for (unsigned long colrow = 0; colrow < map_width * map_height; colrow++)
            {
                if (line[colrow] == '0')
                {
                    map.push_back(0);
                }
                else
                {
                    map.push_back(100);
                }
            }

            map_file.close();

            map_msg.header.frame_id = "map";
            map_msg.info.width = map_width;
            map_msg.info.height = map_height;
            map_msg.info.resolution = 1.0 / 64.0;
            map_msg.info.origin.position.x = 0.0;
            map_msg.info.origin.position.y = 0.0;
            map_msg.info.origin.position.z = 0.0;
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion q_msg;
            q_msg.x = q.x();
            q_msg.y = q.y();
            q_msg.z = q.z();
            q_msg.w = q.w();
            map_msg.info.origin.orientation = q_msg;
            map_msg.data = map;

            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = 0.0;
            pose_msg.pose.position.y = 0.0;
            pose_msg.pose.position.z = 0.0;
            // quaternion from yaw
            tf2::Quaternion q2;
            q2.setRPY(0.0, 0.0, 0.0);
            geometry_msgs::msg::Quaternion q2_msg;
            q2_msg.x = q2.x();
            q2_msg.y = q2.y();
            q2_msg.z = q2.z();
            q2_msg.w = q2.w();
            pose_msg.pose.orientation = q2_msg;
        }

        void map_timer_callback()
        {
            // Create a transform message
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();         // Time stamp
            transform.header.frame_id = "map";            // Source frame (map)
            transform.child_frame_id = "map_transformed"; // Target frame (map_transformed)

            // Apply the y-axis mirroring transformation
            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;
            tf2::Quaternion q2;
            q2.setRPY(M_PI, 0.0, 0.0);
            geometry_msgs::msg::Quaternion q2_msg;
            q2_msg.x = q2.x();
            q2_msg.y = q2.y();
            q2_msg.z = q2.z();
            q2_msg.w = q2.w();
            transform.transform.rotation = q2_msg;

            // Publish the transform
            tf_broadcaster.sendTransform(transform);

            map_publisher->publish(map_msg);
        }

        void upd_timer_callback()
        {
            char msg[5] = {0};

            if (server.recv(msg, sizeof(msg)) > 0)
            {
                jlb::Value value{msg, sizeof(msg)};
                auto msg = std_msgs::msg::Float32();
                msg.data = value.value;
                publishers.at(value.id)->publish(msg);

                if (value.signal.name == "position_x")
                {
                    pose_msg.pose.position.x = value.value * map_msg.info.resolution;
                    pose_publisher->publish(pose_msg);
                }
                else if (value.signal.name == "position_y")
                {
                    pose_msg.pose.position.y = value.value * map_msg.info.resolution;
                    pose_publisher->publish(pose_msg);
                }
                else if (value.signal.name == "position_theta")
                {
                    tf2::Quaternion q2;
                    q2.setRPY(0.0, 0.0, value.value);
                    geometry_msgs::msg::Quaternion q2_msg;
                    q2_msg.x = q2.x();
                    q2_msg.y = q2.y();
                    q2_msg.z = q2.z();
                    q2_msg.w = q2.w();
                    pose_msg.pose.orientation = q2_msg;
                }
            }
        }
    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jlb::Telemetry>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}