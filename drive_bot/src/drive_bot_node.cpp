#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveClient;
typedef move_base_msgs::MoveBaseGoal MoveGoal;

/* Creates a move base goal with predefined properties for use when generating
   waypoints for the project challenge. */
const MoveGoal generateGoal(const double x, const double y, const double w) {
  MoveGoal goal;
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
  MoveClient ac("move_base", true);

  // Create an array of waypoints
  const MoveGoal w_1 = generateGoal(6.7, 6.7, 1.0);
  const MoveGoal w_2 = generateGoal(4.8, 7.9, 1.0);
  const MoveGoal w_3 = generateGoal(-6.0, 3.1, 1.0);
  const MoveGoal w_4 = generateGoal(-2.0, -2.0, 1.0);
  const MoveGoal w_5 = generateGoal(-3.0, -5.0, 1.0);
  const MoveGoal waypoints[] = {w_1, w_2, w_3, w_4, w_5};

  // Iterate over the waypoints
  const unsigned int size = sizeof(waypoints)/sizeof(waypoints[0]);
  for(int i = 0; i < size; i++) {

  // Wait for the action server to come up
  ROS_INFO("Waiting for the move_base action server");
  ac.waitForServer(ros::Duration(5));
  ROS_INFO("Connected to move_base server");
    ROS_INFO("Sending waypoint %i.", i);

    // Send move base goal and wait for result
    ac.sendGoal(waypoints[i]);
    ac.waitForResult();

    // Evaluate the result from move base action server
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Excellent! Your robot has reached waypoint %i.", i);
    } else {
      ROS_INFO("The robot failed to reach waypoint %i.", i);
      break;
    }
  }

  return 0;
}
