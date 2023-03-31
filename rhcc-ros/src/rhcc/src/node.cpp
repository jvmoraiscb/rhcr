#include "node.h"

Node::Node(Falcon falcon, int rate, ros::NodeHandle n) {
  this->falcon = falcon;
  this->rate = rate;
  // Wait for calibration
  ROS_INFO("Please calibrate the controller: move it around and then press the center button.\n");
  this->falcon.calibrate();

  // ROS objects
  this->pub_pos = n.advertise<geometry_msgs::Vector3>("position_vector", rate);
  this->pub_right_b = n.advertise<std_msgs::Int32>("right_button", rate);
  this->pub_up_b = n.advertise<std_msgs::Int32>("up_button", rate);
  this->pub_center_b = n.advertise<std_msgs::Int32>("center_button", rate);
  this->pub_left_b = n.advertise<std_msgs::Int32>("left_button", rate);

  ROS_INFO("The following topics started:\n");
  ROS_INFO("Publisher:\n");
  ROS_INFO("- /position_vector\n");
  ROS_INFO("- /right_button\n");
  ROS_INFO("- /up_button\n");
  ROS_INFO("- /center_button\n");
  ROS_INFO("- /left_button\n");
  ROS_INFO("Subscriber:\n");
  ROS_INFO("- /force_vector\n");
}

void Node::Run() {
  double x, y, z;
  int button1, button2, button3, button4;
  geometry_msgs::Vector3 pos;
  std_msgs::Int32 right_b;
  std_msgs::Int32 up_b;
  std_msgs::Int32 center_b;
  std_msgs::Int32 left_b;
  ros::Rate loop_rate(this->rate);

  while (ros::ok()) {
    this->falcon.update();
    this->falcon.get(&x, &y, &z, &button1, &button2, &button3, &button4);

    pos.x = (float)x;
    pos.y = (float)y;
    pos.z = (float)z;
    right_b.data = button1;
    up_b.data = button2;
    center_b.data = button3;
    left_b.data = button4;

    // ROS_INFO("x: %.2f, y: %.2f, z: %.2f | b1: %d, b2: %d, b3: %d, b4:%d\n", pos.x, pos.y, pos.z, b1.data, b2.data, b3.data, b4.data);

    this->pub_pos.publish(pos);
    this->pub_right_b.publish(right_b);
    this->pub_up_b.publish(up_b);
    this->pub_center_b.publish(center_b);
    this->pub_left_b.publish(left_b);

    ros::spinOnce();

    loop_rate.sleep();
  }
}