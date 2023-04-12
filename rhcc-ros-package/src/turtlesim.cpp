#include "falcon.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  // Falcon object
  libnifalcon::FalconDevice falconDrivers;
  if (!initialise(&falconDrivers))
    return 1;
  Falcon f = Falcon(&falconDrivers);

  // Wait for calibration
  ROS_INFO("Please calibrate the controller: move it around and then press the center button.\n");
  f.calibrate();

  // ROS objects
  ros::init(argc, argv, "rhcc_turtlesim");
  ros::NodeHandle n;
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
  ros::Rate loop_rate(100);

  ROS_INFO("The following topics started:\n");
  ROS_INFO("Publisher:\n");
  ROS_INFO("- /turtle1/cmd_vel\n");

  geometry_msgs::Twist cmd_vel;

  double x, y, z;
  int button1, button2, button3, button4;

  while (ros::ok()) {
    f.update();
    f.get(&x, &y, &z, &button1, &button2, &button3, &button4);

    cmd_vel.linear.x = z * 2;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = x * -1;

    pub_cmd_vel.publish(cmd_vel);

    ros::spinOnce();

    loop_rate.sleep();
  }
}