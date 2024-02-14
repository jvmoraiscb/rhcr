#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
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
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        double robot_x;
        double robot_y;
        static const int minimap_width = 101;
        static const int minimap_height = 101;
        int minimap[minimap_height][minimap_width];
        void map_callback(const nav_msgs::msg::OccupancyGrid msg){
            float resolution = (float)msg.info.resolution;
            int width = (int)msg.info.width;
            int height = (int)msg.info.height;

            double origin_x = (double) msg.info.origin.position.x;
            double origin_y = (double) msg.info.origin.position.y;

            // The origin of the map is indicative of the cell (0,0). By subtracting the robot's position from the origin, the distance from the robot to the origin is obtained. Dividing this value by the map resolution allows for the identification of the cell representing that distance.
            int robot_x_on_map = (int) (robot_x-origin_x)/resolution;
            int robot_y_on_map = (int) (robot_y-origin_y)/resolution;

            // With the minimap being 101 by 101, after identifying the x and y coordinates of the robot on the map, the designated position is set as the center of the map (map[50][50]).
            int initial_i = robot_y_on_map - (minimap_width-1)/2;
            int initial_j = robot_x_on_map - (minimap_height-1)/2;
            int final_i = initial_i + minimap_height;
            int final_j = initial_j + minimap_width;

            for(int i = 0; i < (minimap_height-1)/2; i++){
                for(int j = 0; j < (minimap_width-1)/2; j++){
                    minimap[i][j] = msg.data[i*initial_i*width+j*initial_j];
                }
            }

        }
        void odom_callback(const nav_msgs::msg::Odometry msg){
            robot_x = msg.pose.pose.position.x;
            robot_y = msg.pose.pose.position.y;
        }

    public:
        Minimap_Node() : Node("Minimap_Node"){
            odom_topic_name = this->declare_parameter<std::string>("odom_topic_name", "odom");
            map_topic_name = this->declare_parameter<std::string>("map_topic_name", "map");
            minimap_topic_name = this->declare_parameter<std::string>("minimap_topic_name", "minimap");
            map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(minimap_topic_name, 10);
            map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_name, 10, std::bind(&Minimap_Node::map_callback, this, std::placeholders::_1));
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_name, 10, std::bind(&Minimap_Node::odom_callback, this, std::placeholders::_1));
        }
};

int main(int argc, char * argv[]){

}