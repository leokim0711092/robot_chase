#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/qos.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

float euler_degree_transform(geometry_msgs::msg::TransformStamped t_st){
        // float x = t_st.transform.rotation.x;
        // float y = t_st.transform.rotation.y;
        // float z = t_st.transform.rotation.z;
        // float w = t_st.transform.rotation.w; 
        float x = t_st.transform.translation.x;
        float y = t_st.transform.translation.y;

    // return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
    return atan2(y, x);
}

float error_dist(geometry_msgs::msg::TransformStamped t_st){
        float x = t_st.transform.translation.x;
        float y = t_st.transform.translation.y;

    return std::sqrt(std::pow(x,2) + std::pow(y,2));
    // 
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tf_transform_node");

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    rclcpp::WallRate rate(1.0); // 1Hz
    float error_distance = 0;
    float error_yaw = 0;
    geometry_msgs::msg::Twist vel;
    float kp_yaw = 1.2;
    float kp_distance = 1.2;
    auto pub_rick = node->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);


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

            error_distance = error_dist(transform_stamped);
            error_yaw = euler_degree_transform(transform_stamped);

            vel.linear.x = error_distance*kp_distance;
            vel.angular.y = error_yaw*kp_yaw;

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

