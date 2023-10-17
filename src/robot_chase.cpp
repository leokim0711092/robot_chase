#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/qos.hpp"
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

std::shared_ptr<nav_msgs::msg::Odometry> odom_msg;

float euler_degree_transform(const nav_msgs::msg::Odometry::SharedPtr t_st){
        float x = t_st->pose.pose.orientation.x;
        float y = t_st->pose.pose.orientation.y;
        float z = t_st->pose.pose.orientation.z;
        float w = t_st->pose.pose.orientation.w; 


    return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
}

float error_dist(geometry_msgs::msg::TransformStamped t_st){
        float x = t_st.transform.translation.x;
        float y = t_st.transform.translation.y;

    return std::sqrt(std::pow(x,2) + std::pow(y,2));
    // 
}
// void callback(const nav_msgs::msg::Odometry::SharedPtr msg){
//     odom_msg = msg;
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tf_transform_node");

    // rmw_qos_profile_t qos_ = rmw_qos_profile_default; 

    // qos_.depth = 10;
    // qos_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    // qos_.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    rclcpp::WallRate rate(1.0); // 1Hz
    float error_distance = 0;
    float error_yaw = 0;
    geometry_msgs::msg::Twist vel;
    float kp_yaw = 0.5;
    float kp_distance = 0.5;
    float error_y = 0;
    float error_x = 0;
    auto pub_rick = node->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);
    // auto sub_rick = node->create_subscription<nav_msgs::msg::Odometry>("rick/odom", 10, &callback);


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
            // error_yaw = euler_degree_transform(transform_stamped);
            error_y = std::fabs(transform_stamped.transform.translation.y);
            error_x = std::fabs(transform_stamped.transform.translation.x); 

            vel.linear.x = error_distance*kp_distance > 0.6 ? 0.6:error_distance*kp_distance ;

            // // if(error_yaw*kp_yaw > 0) vel.angular.z = error_yaw*kp_yaw > 1 ? 1 : std::fabs(error_yaw*kp_yaw);
            // // else vel.angular.z = error_yaw*kp_yaw < -1 ? -1 : std::fabs(error_yaw*kp_yaw);


            if(transform_stamped.transform.translation.x > 0 && transform_stamped.transform.translation.y > 0) 
            error_yaw = atan2(error_y, error_x) ;

            else if (transform_stamped.transform.translation.x > 0 && transform_stamped.transform.translation.y < 0) 
            error_yaw = (atan2(error_y, error_x)*-1);

            else if (transform_stamped.transform.translation.x < 0 && transform_stamped.transform.translation.y > 0) 
            error_yaw = (atan2(error_x, error_y) + 1.57);
            else error_yaw = ((atan2(error_x, error_y)*-1) - 1.57);
            if(error_yaw >3.14 || error_yaw < -3.14 ){
                error_yaw = error_yaw*-1;
            }

            vel.angular.z  = error_yaw*kp_yaw;

            if (vel.angular.z > 1) vel.angular.z = 1;
            if (vel.angular.z < -1)vel.angular.z = -1;



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

