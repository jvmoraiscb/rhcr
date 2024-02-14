#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Minimap_Node : public rclcpp::Node{
    private:
        std::string odom_topic_name;
        std::string map_topic_name;
        std::string minimap_topic_name;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        int position[2] = {0, 0};
        void map_callback(const nav_msgs::msg::OccupancyGrid msg){
            for(int i = position[0] - 50; i < position)
        }
        

    public:
        Minimap_Node() : Node("Minimap_Node"){
            odom_topic_name = this->declare_parameter<std::string>("odom_topic_name", "odom");
            map_topic_name = this->declare_parameter<std::string>("map_topic_name", "map");
            minimap_topic_name = this->declare_parameter<std::string>("minimap_topic_name", "minimap");
            map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(minimap_topic_name, 10);
            map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_name, 10, std::bind(&Minimap_Node::map_callback, this, std::placeholders::_1));
        }
};

int main(int argc, char * argv[]){

}