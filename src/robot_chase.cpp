#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
using namespace std::chrono_literals;

nav_msgs::msg::Odometry rick_odom_msg;
nav_msgs::msg::Odometry morty_odom_msg;

float euler_degree_transform(nav_msgs::msg::Odometry t_st){
        float x = t_st.pose.pose.orientation.x;
        float y = t_st.pose.pose.orientation.y;
        float z = t_st.pose.pose.orientation.z;
        float w = t_st.pose.pose.orientation.w; 


    return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
}

float error_dist(nav_msgs::msg::Odometry t_st){
        float x = t_st.pose.pose.position.x;
        float y = t_st.pose.pose.position.y;

    return std::sqrt(std::pow(x,2) + std::pow(y,2));
}
void rick_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    rick_odom_msg = *msg;
}

void morty_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    morty_odom_msg = *msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tf_transform_node");

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    rclcpp::WallRate rate(1.0); // 1Hz
    float error_distance = 0;
    float error_yaw = 0;
    geometry_msgs::msg::Twist vel;
    float kp_yaw = 0.9;
    float kp_distance = 0.5;
    float error_y = 0;
    float error_x = 0;
    float fab_error_y = 0;
    float fab_error_x = 0;
    auto pub_rick = node->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);
    auto sub_rick = node->create_subscription<nav_msgs::msg::Odometry>("rick/odom", qos_profile, &rick_callback);
    auto sub_morty = node->create_subscription<nav_msgs::msg::Odometry>("morty/odom", qos_profile, &morty_callback);
    float result = 0;


    while (rclcpp::ok())
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        try
        {
            // Lookup transform
            transform_stamped = tf_buffer.lookupTransform("rick/base_link", "morty/base_link", 
                                                          tf2::TimePointZero);

            // Do something with the transform
            RCLCPP_INFO(node->get_logger(), "Translation x: %f, y: %f, z: %f, Rotation x: %f, y: %f, z: %f, w: %f",
                        transform_stamped.transform.translation.x,
                        transform_stamped.transform.translation.y,
                        transform_stamped.transform.translation.z,
                        transform_stamped.transform.rotation.x,
                        transform_stamped.transform.rotation.y,
                        transform_stamped.transform.rotation.z,
                        transform_stamped.transform.rotation.w                        
                        );
            
            result = euler_degree_transform(rick_odom_msg);
            std::cout << "Received result: " << result << std::endl;

            error_distance = error_dist(morty_odom_msg);

            // error_yaw = euler_degree_transform(transform_stamped);
            fab_error_x = std::fabs(morty_odom_msg.pose.pose.position.x - rick_odom_msg.pose.pose.position.x);
            fab_error_y = std::fabs(morty_odom_msg.pose.pose.position.y - rick_odom_msg.pose.pose.position.y); 
            error_x = morty_odom_msg.pose.pose.position.x - rick_odom_msg.pose.pose.position.x;
            error_y = morty_odom_msg.pose.pose.position.y - rick_odom_msg.pose.pose.position.y; 

            vel.linear.x = error_distance*kp_distance > 0.5 ? 0.5:error_distance*kp_distance ;

            // // if(error_yaw*kp_yaw > 0) vel.angular.z = error_yaw*kp_yaw > 1 ? 1 : std::fabs(error_yaw*kp_yaw);
            // // else vel.angular.z = error_yaw*kp_yaw < -1 ? -1 : std::fabs(error_yaw*kp_yaw);

            
            // float result = 0;
            // result = euler_degree_transform(rick_odom_msg);
            // std::cout << "Received result: " << result << std::endl;
            std::cout << "Received Arctan: " << atan2(fab_error_y, fab_error_x) << std::endl;
            std::cout << "X error: " << fab_error_x << "Y error: " << fab_error_y << std::endl;

            if(error_x > 0 && error_y > 0) 
            error_yaw = atan2(fab_error_y, fab_error_x) - euler_degree_transform(rick_odom_msg) ;

            else if (error_x > 0 && error_y < 0) 
            error_yaw = (atan2(fab_error_y, fab_error_x)*-1) -euler_degree_transform(rick_odom_msg);

            else if (error_x < 0 && error_y > 0) 
            error_yaw = (atan2(fab_error_y, fab_error_x) + 1.57) -euler_degree_transform(rick_odom_msg);

            else error_yaw = ((atan2(fab_error_y, fab_error_x)*-1) - 1.57) -euler_degree_transform(rick_odom_msg);

            if(error_yaw >3.14 || error_yaw < -3.14 ){
                error_yaw = error_yaw*-1;
            }

            if (error_yaw > 0.1) {
                vel.linear.x = 0.2;
            }
            std::cout << "Received error_yaw: " << error_yaw << std::endl;

            vel.angular.z  = error_yaw*kp_yaw;

            if (vel.angular.z > 3) vel.angular.z = 3;
            if (vel.angular.z < -3)vel.angular.z = -3;



            std::cout << "x: " << vel.linear.x <<std::endl;
            std::cout << "z: " << vel.angular.z <<std::endl;

            pub_rick->publish(vel);
        }
        catch (tf2::LookupException &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Transform lookup failed: %s", e.what());
        }
        catch (tf2::ExtrapolationException &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Transform extrapolation failed: %s", e.what());
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

