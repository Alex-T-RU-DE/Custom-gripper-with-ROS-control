#include "ros/ros.h"
#include "youbot_gripper/grip_service.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_gripper_client");
  if (argc != 2)
  {
    ROS_INFO("usage: enter desired position 0-380");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<youbot_gripper::grip_service>("youbot_gripper");
  youbot_gripper::grip_service srv;
  srv.request.pos = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Result: %ld", (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service youbot_gripper");
    return 1;
  }

  return 0;
}
