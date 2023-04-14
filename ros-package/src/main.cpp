#include "node.h"

int main(int argc, char** argv) {
  // Falcon device object
  libnifalcon::FalconDevice falconDevice;
  if (!initialise(&falconDevice))
    return 1;

  ros::init(argc, argv, "rhcc_main");
  ros::NodeHandle nodeHandle;

  Node n = Node(Falcon(&falconDevice), 100, nodeHandle);
  n.Run();

  return 2;
}