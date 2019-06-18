#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* Creates a move base goal with predefined properties for use when generating
   waypoints for the project challenge. */
const move_base_msgs::MoveBaseGoal generateGoal(const double x,
                                                const double y,
                                                const double w) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = w;
  return goal;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drive_bot_node");
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up
  ROS_INFO("Waiting for the move_base action server");
  ac.waitForServer(ros::Duration(5));
  ROS_INFO("Connected to move_base server");

  // Create the first waypoint
  const move_base_msgs::MoveBaseGoal waypoint_1 = generateGoal(6.6, 6.6, 3.0);
  ROS_INFO("Sending waypoint_1.");

  // Send move base goal and wait for result
  ac.sendGoal(waypoint_1);
  ac.waitForResult();

  // Evaluate the result from move base action server
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Excellent! Your robot has reached the waypoint.");
  else
    ROS_INFO("The robot failed to reach the waypoint.");
  return 0;
}
