#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include "act_file.h"
using namespace std;

vector<float> data_from_file;

//char *filePath3 = "/home/tma/catkin_ws/src/odom_sub/file/b.csv";
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
  ros::init(argc, argv, "goals");
  ros::NodeHandle n;
  bool receive = false;
  float goal_x,goal_y,goal_z,goal_w;
  int size_num = 0;

  ros::Rate loop_rate(10);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  size_num = count_line(filePath3);
  int block = size_num / 4;
  //ROS_INFO("size_num = %d", size_num);
  SaveToArray(size_num, filePath2);
  
  while(ros::ok())
  {
    if(!receive)
    {
        receive = true;
        int j = 0;
        while(j < size_num)
        {
          move_base_msgs::MoveBaseGoal goal;


          //we'll send a goal to the robot to move 1 meter forward
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();

          goal.target_pose.pose.position.x = data_from_file[j];
          goal.target_pose.pose.position.y = data_from_file[j+1];
          goal.target_pose.pose.orientation.w = data_from_file[j+2];
          goal.target_pose.pose.orientation.z = data_from_file[j+3];




          ROS_INFO("J: %d", j);
          ROS_INFO("x1 = %f", data_from_file[j]);
          ROS_INFO("y1 = %f", data_from_file[j+1]);
          ROS_INFO("z1 = %f", data_from_file[j+2]);
          ROS_INFO("w1 = %f", data_from_file[j+3]);
          ac.sendGoal(goal);
          ROS_INFO("Sending goal");
          ac.waitForResult();

          if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ;
          }
          ROS_INFO("GOAL FINISH");
          j = j + 4;

        }
    }
  ros::spinOnce();
  loop_rate.sleep();
  }

  return 0;
}
