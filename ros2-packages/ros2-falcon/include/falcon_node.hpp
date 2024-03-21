#ifndef FALCON_NODE_HPP
#define FALCON_NODE_HPP

#include "falcon.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class Falcon_Node : public rclcpp::Node {
   public:
    Falcon_Node(Falcon* falcon, bool* debug_mode);

   private:
    Falcon* falcon_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    bool* debug_mode_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_vector_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_button_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr up_button_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr center_button_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_button_pub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_vector_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rgb_vector_sub;

    void timer_callback();
    void force_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void rgb_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
};

#endif
