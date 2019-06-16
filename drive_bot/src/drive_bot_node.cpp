#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "drive_bot_node");

  ros::spin();
  return 0;
}
