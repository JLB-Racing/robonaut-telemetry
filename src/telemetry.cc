#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robonaut_telemetry/msg/logic1.hpp>
#include <robonaut_telemetry/msg/logic2.hpp>
#include <robonaut_telemetry/msg/logic3.hpp>
#include <robonaut_telemetry/msg/measurements1.hpp>
#include <robonaut_telemetry/msg/measurements3.hpp>
#include <robonaut_telemetry/msg/measurements4.hpp>
#include <robonaut_telemetry/msg/measurements5.hpp>
#include <robonaut_telemetry/msg/odometry1.hpp>
#include <robonaut_telemetry/msg/odometry2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "JLB/common.hxx"
#include "JLB/udp.hxx"
#include "jlb-binutil.h"

// measurements 2 is the same as measurements1
namespace robonaut_telemetry
{
    namespace msg
    {
        using Measurements2 = Measurements1;
    }  // namespace msg
}  // namespace robonaut_telemetry

namespace jlb
{

    class Telemetry : public rclcpp::Node
    {
    public:
        Telemetry() : Node("telemetry"), server(RECEIVER_ADDRESS, RECEIVER_PORT), tf_broadcaster(this)
        {
            RCLCPP_INFO(get_logger(), "node started.");

            parse_map("competition.map");

            map_publisher  = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
            pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
            map_timer      = create_wall_timer(std::chrono::milliseconds(16), std::bind(&Telemetry::map_timer_callback, this));
            udp_timer      = create_wall_timer(std::chrono::milliseconds(1), std::bind(&Telemetry::upd_timer_callback, this));

            measurements1_publisher = create_publisher<robonaut_telemetry::msg::Measurements1>("measurements1", 10);
            measurements2_publisher = create_publisher<robonaut_telemetry::msg::Measurements2>("measurements2", 10);
            measurements3_publisher = create_publisher<robonaut_telemetry::msg::Measurements3>("measurements3", 10);
            measurements4_publisher = create_publisher<robonaut_telemetry::msg::Measurements4>("measurements4", 10);
            measurements5_publisher = create_publisher<robonaut_telemetry::msg::Measurements5>("measurements5", 10);
            odometry1_publisher     = create_publisher<robonaut_telemetry::msg::Odometry1>("odometry1", 10);
            odometry2_publisher     = create_publisher<robonaut_telemetry::msg::Odometry2>("odometry2", 10);
            logic1_publisher        = create_publisher<robonaut_telemetry::msg::Logic1>("logic1", 10);
            logic2_publisher        = create_publisher<robonaut_telemetry::msg::Logic2>("logic2", 10);
            logic3_publisher        = create_publisher<robonaut_telemetry::msg::Logic3>("logic3", 10);
        }
        ~Telemetry() { RCLCPP_INFO(get_logger(), "node stopped."); }

    private:
        jlb_rx_t                                                                    jlb_rx_t;
        UDPServer                                                                   server;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers;

        std::vector<int8_t>                                           map;
        nav_msgs::msg::OccupancyGrid                                  map_msg;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_publisher;
        tf2_ros::StaticTransformBroadcaster                           tf_broadcaster;
        geometry_msgs::msg::PoseStamped                               pose_msg;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;

        rclcpp::Publisher<robonaut_telemetry::msg::Measurements1>::SharedPtr measurements1_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Measurements2>::SharedPtr measurements2_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Measurements3>::SharedPtr measurements3_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Measurements4>::SharedPtr measurements4_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Measurements5>::SharedPtr measurements5_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Odometry1>::SharedPtr     odometry1_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Odometry2>::SharedPtr     odometry2_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Logic1>::SharedPtr        logic1_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Logic2>::SharedPtr        logic2_publisher;
        rclcpp::Publisher<robonaut_telemetry::msg::Logic3>::SharedPtr        logic3_publisher;

        rclcpp::TimerBase::SharedPtr map_timer;
        rclcpp::TimerBase::SharedPtr udp_timer;

        void parse_map(const std::string &filename)
        {
            std::ifstream map_file;
            auto          robonaut_telemetry_share_dir = ament_index_cpp::get_package_share_directory("robonaut_telemetry");
            auto          file_path                    = robonaut_telemetry_share_dir + "/maps/" + filename;

            map_file.open(file_path);
            std::string line;
            std::getline(map_file, line);
            if (!map_file.is_open())
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
                if (line[colrow] == '0') { map.push_back(0); }
                else { map.push_back(100); }
            }

            map_file.close();

            map_msg.header.frame_id        = "map";
            map_msg.info.width             = map_width;
            map_msg.info.height            = map_height;
            map_msg.info.resolution        = px_to_m(1.0);
            map_msg.info.origin.position.x = 0.0;
            map_msg.info.origin.position.y = 0.0;
            map_msg.info.origin.position.z = 0.0;
            double          roll           = 0.0;
            double          pitch          = 0.0;
            double          yaw            = 0.0;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion q_msg;
            q_msg.x                         = q.x();
            q_msg.y                         = q.y();
            q_msg.z                         = q.z();
            q_msg.w                         = q.w();
            map_msg.info.origin.orientation = q_msg;
            map_msg.data                    = map;

            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = 0.0;
            pose_msg.pose.position.y = 0.0;
            pose_msg.pose.position.z = 0.0;
            // quaternion from yaw
            tf2::Quaternion q2;
            q2.setRPY(0.0, 0.0, 0.0);
            geometry_msgs::msg::Quaternion q2_msg;
            q2_msg.x                  = q2.x();
            q2_msg.y                  = q2.y();
            q2_msg.z                  = q2.z();
            q2_msg.w                  = q2.w();
            pose_msg.pose.orientation = q2_msg;
        }

        void map_timer_callback()
        {
            // Create a transform message
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp    = this->now();        // Time stamp
            transform.header.frame_id = "map";              // Source frame (map)
            transform.child_frame_id  = "map_transformed";  // Target frame (map_transformed)

            // Apply the y-axis mirroring transformation
            transform.transform.translation.x = 4.8;
            transform.transform.translation.y = 4.8;
            transform.transform.translation.z = 0.0;
            tf2::Quaternion q2;
            q2.setRPY(M_PI, 0.0, -M_PI / 2.0);
            geometry_msgs::msg::Quaternion q2_msg;
            q2_msg.x                     = q2.x();
            q2_msg.y                     = q2.y();
            q2_msg.z                     = q2.z();
            q2_msg.w                     = q2.w();
            transform.transform.rotation = q2_msg;

            // Create a transform message
            geometry_msgs::msg::TransformStamped transform_pose;
            transform_pose.header.stamp    = this->now();  // Time stamp
            transform_pose.header.frame_id = "map";        // Source frame (map)
            transform_pose.child_frame_id  = "pose";       // Target frame (map_transformed)

            // Apply the y-axis mirroring transformation
            transform_pose.transform.translation.x = pose_msg.pose.position.x;
            transform_pose.transform.translation.y = pose_msg.pose.position.y;
            transform_pose.transform.translation.z = 0.0;
            tf2::Quaternion q3{pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w};
            auto            yaw   = 0.0;
            auto            pitch = 0.0;
            auto            roll  = 0.0;
            tf2::Matrix3x3  m(q3);
            m.getRPY(roll, pitch, yaw);

            q3.setRPY(roll + M_PI, pitch, yaw);
            geometry_msgs::msg::Quaternion q3_msg;
            q3_msg.x                          = q3.x();
            q3_msg.y                          = q3.y();
            q3_msg.z                          = q3.z();
            q3_msg.w                          = q3.w();
            transform_pose.transform.rotation = q3_msg;

            // Publish the transform
            tf_broadcaster.sendTransform(transform_pose);

            // Publish the transform
            tf_broadcaster.sendTransform(transform);

            map_publisher->publish(map_msg);
        }

        void upd_timer_callback()
        {
            char data[1500]     = {0};
            auto bytes_received = server.recv(data, sizeof(data));

            if (bytes_received > 0)
            {
                auto idx = 0;
                while (idx < bytes_received)
                {
                    uint32_t can_id      = data[idx];
                    uint8_t  dlc         = data[idx + 1];
                    uint32_t received_id = jlb_Receive(&jlb_rx_t, reinterpret_cast<uint8_t *>(data + idx + 2), can_id, dlc);

                    switch (received_id)
                    {
                        case measurements_1_CANID:
                        {
                            robonaut_telemetry::msg::Measurements1 measurements1_msg;
                            measurements1_msg.header.stamp   = this->now();
                            measurements1_msg.line_sensor_1  = jlb_rx_t.measurements_1.line_sensor_1;
                            measurements1_msg.line_sensor_2  = jlb_rx_t.measurements_1.line_sensor_2;
                            measurements1_msg.line_sensor_3  = jlb_rx_t.measurements_1.line_sensor_3;
                            measurements1_msg.line_sensor_4  = jlb_rx_t.measurements_1.line_sensor_4;
                            measurements1_msg.line_sensor_5  = jlb_rx_t.measurements_1.line_sensor_5;
                            measurements1_msg.line_sensor_6  = jlb_rx_t.measurements_1.line_sensor_6;
                            measurements1_msg.line_sensor_7  = jlb_rx_t.measurements_1.line_sensor_7;
                            measurements1_msg.line_sensor_8  = jlb_rx_t.measurements_1.line_sensor_8;
                            measurements1_msg.line_sensor_9  = jlb_rx_t.measurements_1.line_sensor_9;
                            measurements1_msg.line_sensor_10 = jlb_rx_t.measurements_1.line_sensor_10;
                            measurements1_msg.line_sensor_11 = jlb_rx_t.measurements_1.line_sensor_11;
                            measurements1_msg.line_sensor_12 = jlb_rx_t.measurements_1.line_sensor_12;
                            measurements1_msg.line_sensor_13 = jlb_rx_t.measurements_1.line_sensor_13;
                            measurements1_msg.line_sensor_14 = jlb_rx_t.measurements_1.line_sensor_14;
                            measurements1_msg.line_sensor_15 = jlb_rx_t.measurements_1.line_sensor_15;
                            measurements1_msg.line_sensor_16 = jlb_rx_t.measurements_1.line_sensor_16;
                            measurements1_msg.line_sensor_17 = jlb_rx_t.measurements_1.line_sensor_17;
                            measurements1_msg.line_sensor_18 = jlb_rx_t.measurements_1.line_sensor_18;
                            measurements1_msg.line_sensor_19 = jlb_rx_t.measurements_1.line_sensor_19;
                            measurements1_msg.line_sensor_20 = jlb_rx_t.measurements_1.line_sensor_20;
                            measurements1_msg.line_sensor_21 = jlb_rx_t.measurements_1.line_sensor_21;
                            measurements1_msg.line_sensor_22 = jlb_rx_t.measurements_1.line_sensor_22;
                            measurements1_msg.line_sensor_23 = jlb_rx_t.measurements_1.line_sensor_23;
                            measurements1_msg.line_sensor_24 = jlb_rx_t.measurements_1.line_sensor_24;
                            measurements1_msg.line_sensor_25 = jlb_rx_t.measurements_1.line_sensor_25;
                            measurements1_msg.line_sensor_26 = jlb_rx_t.measurements_1.line_sensor_26;
                            measurements1_msg.line_sensor_27 = jlb_rx_t.measurements_1.line_sensor_27;
                            measurements1_msg.line_sensor_28 = jlb_rx_t.measurements_1.line_sensor_28;
                            measurements1_msg.line_sensor_29 = jlb_rx_t.measurements_1.line_sensor_29;
                            measurements1_msg.line_sensor_30 = jlb_rx_t.measurements_1.line_sensor_30;
                            measurements1_msg.line_sensor_31 = jlb_rx_t.measurements_1.line_sensor_31;
                            measurements1_msg.line_sensor_32 = jlb_rx_t.measurements_1.line_sensor_32;
                            measurements1_publisher->publish(measurements1_msg);
                            break;
                        }
                        case measurements_2_CANID:
                        {
                            robonaut_telemetry::msg::Measurements2 measurements2_msg;
                            measurements2_msg.header.stamp   = this->now();
                            measurements2_msg.line_sensor_1  = jlb_rx_t.measurements_2.line_sensor_1;
                            measurements2_msg.line_sensor_2  = jlb_rx_t.measurements_2.line_sensor_2;
                            measurements2_msg.line_sensor_3  = jlb_rx_t.measurements_2.line_sensor_3;
                            measurements2_msg.line_sensor_4  = jlb_rx_t.measurements_2.line_sensor_4;
                            measurements2_msg.line_sensor_5  = jlb_rx_t.measurements_2.line_sensor_5;
                            measurements2_msg.line_sensor_6  = jlb_rx_t.measurements_2.line_sensor_6;
                            measurements2_msg.line_sensor_7  = jlb_rx_t.measurements_2.line_sensor_7;
                            measurements2_msg.line_sensor_8  = jlb_rx_t.measurements_2.line_sensor_8;
                            measurements2_msg.line_sensor_9  = jlb_rx_t.measurements_2.line_sensor_9;
                            measurements2_msg.line_sensor_10 = jlb_rx_t.measurements_2.line_sensor_10;
                            measurements2_msg.line_sensor_11 = jlb_rx_t.measurements_2.line_sensor_11;
                            measurements2_msg.line_sensor_12 = jlb_rx_t.measurements_2.line_sensor_12;
                            measurements2_msg.line_sensor_13 = jlb_rx_t.measurements_2.line_sensor_13;
                            measurements2_msg.line_sensor_14 = jlb_rx_t.measurements_2.line_sensor_14;
                            measurements2_msg.line_sensor_15 = jlb_rx_t.measurements_2.line_sensor_15;
                            measurements2_msg.line_sensor_16 = jlb_rx_t.measurements_2.line_sensor_16;
                            measurements2_msg.line_sensor_17 = jlb_rx_t.measurements_2.line_sensor_17;
                            measurements2_msg.line_sensor_18 = jlb_rx_t.measurements_2.line_sensor_18;
                            measurements2_msg.line_sensor_19 = jlb_rx_t.measurements_2.line_sensor_19;
                            measurements2_msg.line_sensor_20 = jlb_rx_t.measurements_2.line_sensor_20;
                            measurements2_msg.line_sensor_21 = jlb_rx_t.measurements_2.line_sensor_21;
                            measurements2_msg.line_sensor_22 = jlb_rx_t.measurements_2.line_sensor_22;
                            measurements2_msg.line_sensor_23 = jlb_rx_t.measurements_2.line_sensor_23;
                            measurements2_msg.line_sensor_24 = jlb_rx_t.measurements_2.line_sensor_24;
                            measurements2_msg.line_sensor_25 = jlb_rx_t.measurements_2.line_sensor_25;
                            measurements2_msg.line_sensor_26 = jlb_rx_t.measurements_2.line_sensor_26;
                            measurements2_msg.line_sensor_27 = jlb_rx_t.measurements_2.line_sensor_27;
                            measurements2_msg.line_sensor_28 = jlb_rx_t.measurements_2.line_sensor_28;
                            measurements2_msg.line_sensor_29 = jlb_rx_t.measurements_2.line_sensor_29;
                            measurements2_msg.line_sensor_30 = jlb_rx_t.measurements_2.line_sensor_30;
                            measurements2_msg.line_sensor_31 = jlb_rx_t.measurements_2.line_sensor_31;
                            measurements2_msg.line_sensor_32 = jlb_rx_t.measurements_2.line_sensor_32;
                            measurements2_publisher->publish(measurements2_msg);
                            break;
                        }
                        case measurements_3_CANID:
                        {
                            robonaut_telemetry::msg::Measurements3 measurements3_msg;
                            measurements3_msg.header.stamp       = this->now();
                            measurements3_msg.angular_velocity_x = jlb_rx_t.measurements_3.angular_velocity_x_phys;
                            measurements3_msg.angular_velocity_y = jlb_rx_t.measurements_3.angular_velocity_y_phys;
                            measurements3_msg.angular_velocity_z = jlb_rx_t.measurements_3.angular_velocity_z_phys;
                            measurements3_publisher->publish(measurements3_msg);
                            break;
                        }
                        case measurements_4_CANID:
                        {
                            robonaut_telemetry::msg::Measurements4 measurements4_msg;
                            measurements4_msg.header.stamp          = this->now();
                            measurements4_msg.linear_acceleration_x = jlb_rx_t.measurements_4.linear_acceleration_x_phys;
                            measurements4_msg.linear_acceleration_y = jlb_rx_t.measurements_4.linear_acceleration_y_phys;
                            measurements4_msg.linear_acceleration_z = jlb_rx_t.measurements_4.linear_acceleration_z_phys;
                            measurements4_publisher->publish(measurements4_msg);
                            break;
                        }
                        case measurements_5_CANID:
                        {
                            robonaut_telemetry::msg::Measurements5 measurements5_msg;
                            measurements5_msg.header.stamp  = this->now();
                            measurements5_msg.duty_cycle    = jlb_rx_t.measurements_5.duty_cycle_phys;
                            measurements5_msg.motor_current = jlb_rx_t.measurements_5.motor_current_phys;
                            measurements5_msg.object_range  = jlb_rx_t.measurements_5.object_range_phys;
                            measurements5_msg.wheel_rpm     = jlb_rx_t.measurements_5.wheel_rpm_phys;
                            measurements5_publisher->publish(measurements5_msg);
                            break;
                        }
                        case odometry_1_CANID:
                        {
                            robonaut_telemetry::msg::Odometry1 odometry1_msg;
                            odometry1_msg.header.stamp = this->now();
                            odometry1_msg.position_x   = jlb_rx_t.odometry_1.position_x_phys;
                            odometry1_msg.position_y   = jlb_rx_t.odometry_1.position_y_phys;
                            odometry1_msg.orientation  = jlb_rx_t.odometry_1.orientation_phys;
                            odometry1_publisher->publish(odometry1_msg);

                            pose_msg.pose.position.x = odometry1_msg.position_x;
                            pose_msg.pose.position.y = odometry1_msg.position_y;
                            tf2::Quaternion q2;
                            q2.setRPY(0.0, 0.0, odometry1_msg.orientation);
                            geometry_msgs::msg::Quaternion q2_msg;
                            q2_msg.x                  = q2.x();
                            q2_msg.y                  = q2.y();
                            q2_msg.z                  = q2.z();
                            q2_msg.w                  = q2.w();
                            pose_msg.pose.orientation = q2_msg;
                            pose_publisher->publish(pose_msg);
                            break;
                        }
                        case odometry_2_CANID:
                        {
                            robonaut_telemetry::msg::Odometry2 odometry2_msg;
                            odometry2_msg.header.stamp       = this->now();
                            odometry2_msg.linear_velocity_x  = jlb_rx_t.odometry_2.linear_velocity_x_phys;
                            odometry2_msg.angular_velocity_z = jlb_rx_t.odometry_2.angular_velocity_z_phys;
                            odometry2_publisher->publish(odometry2_msg);
                            break;
                        }
                        case logic_1_CANID:
                        {
                            robonaut_telemetry::msg::Logic1 logic1_msg;
                            logic1_msg.header.stamp      = this->now();
                            logic1_msg.target_speed      = jlb_rx_t.logic_1.target_speed_phys;
                            logic1_msg.target_angle      = jlb_rx_t.logic_1.target_angle_phys;
                            logic1_msg.cross_track_error = jlb_rx_t.logic_1.cross_track_error_phys;
                            logic1_msg.heading_error     = jlb_rx_t.logic_1.heading_error_phys;
                            logic1_publisher->publish(logic1_msg);
                            break;
                        }
                        case logic_2_CANID:
                        {
                            robonaut_telemetry::msg::Logic2 logic2_msg;
                            logic2_msg.header.stamp      = this->now();
                            logic2_msg.distance_traveled = jlb_rx_t.logic_2.distance_traveled_phys;
                            logic2_msg.labyrinth_state   = jlb_rx_t.logic_2.labyrinth_state;
                            logic2_msg.fast_state        = jlb_rx_t.logic_2.fast_state;
                            logic2_msg.next_node         = jlb_rx_t.logic_2.next_node;
                            logic2_msg.previous_node     = jlb_rx_t.logic_2.previous_node;
                            logic2_msg.direction         = jlb_rx_t.logic_2.direction;
                            logic2_msg.mission           = jlb_rx_t.logic_2.mission;
                            logic2_publisher->publish(logic2_msg);
                            break;
                        }
                        case logic_3_CANID:
                        {
                            robonaut_telemetry::msg::Logic3 logic3_msg;
                            logic3_msg.header.stamp        = this->now();
                            logic3_msg.ang_error_norm      = jlb_rx_t.logic_3.ang_error_norm_phys;
                            logic3_msg.dist_error_norm     = jlb_rx_t.logic_3.dist_error_norm_phys;
                            logic3_msg.line_position_rear  = jlb_rx_t.logic_3.line_position_rear_phys;
                            logic3_msg.line_position_front = jlb_rx_t.logic_3.line_position_front_phys;
                            logic3_msg.at_cross_section    = jlb_rx_t.logic_3.at_cross_section;
                            logic3_msg.under_gate          = jlb_rx_t.logic_3.under_gate;
                            logic3_publisher->publish(logic3_msg);
                            break;
                        }

                        default:
                            break;
                    }
                    idx += dlc + 2;
                }
            }
        }
    };
}  // namespace jlb

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jlb::Telemetry>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}