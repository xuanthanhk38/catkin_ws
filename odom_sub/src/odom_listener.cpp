#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <iostream>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "act_file.h"
using namespace std;


//void WriteToFile(float pos_x, float pos_y, float orien_z, float orien_w);
//void SaveToArray(int size);
//void copy_file(char *sourcefile, char *destinationfile);
//int count_line(char *sourcefile);

//char *filePath = "/home/tma/catkin_ws/src/odom_sub/file/a.csv";
//char *filePath2 = "/home/tma/catkin_ws/src/odom_sub/file/b.csv";
//char *filePath3 = "/home/tma/catkin_ws/src/odom_sub/file/c.csv";


//float data_from_file[300];


vector<float> data_from_file;

void chatterCallback(const nav_msgs::Odometry& msg)
{
  int size_num = 0;
  static float prev_x = msg.pose.pose.position.x;
  static float prev_y = msg.pose.pose.position.y;
  static int count = 0;

  float goal_x,goal_y,goal_z,goal_w;

  float x = 0, y = 0;
  float oren_x = 0, oren_y = 0, oren_z =0, oren_w = 0;
  static float distance_moved = 0.0;

  x = msg.pose.pose.position.x;

  y = msg.pose.pose.position.y;

  oren_x = msg.pose.pose.orientation.x;
  oren_y = msg.pose.pose.orientation.y;
  oren_z = msg.pose.pose.orientation.z;
  oren_w = msg.pose.pose.orientation.w;



  
  distance_moved += sqrt(pow((x - prev_x), 2) + pow((y- prev_y),2));
  if(distance_moved != 0)
  {

    prev_x = msg.pose.pose.position.x;
    prev_y = msg.pose.pose.position.y;
  }

  if(distance_moved >= 0.05)
  {
    ROS_INFO("distance_moved : [%f]", distance_moved);
    count++;
    distance_moved = 0;
    ROS_INFO("----------------------------");
    ROS_INFO("count : [%d]", count);
    ROS_INFO("Update distance_moved");
    ROS_INFO("Position-> x: [%f]", x);
    ROS_INFO("Position-> y: [%f]", y);
    ROS_INFO("orientation-> oren_x: [%f]", oren_x);
    ROS_INFO("orientation-> oren_y: [%f]", oren_y);
    ROS_INFO("orientation-> oren_z: [%f]", oren_z);
    ROS_INFO("orientation-> oren_w: [%f]", oren_w);

    WriteToFile(x, y, oren_z, oren_w);
    copy_file(filePath, filePath2);
    size_num = count_line(filePath2);

    ROS_INFO("size_num: %d", size_num);
    SaveToArray(size_num, filePath2); // read data from file and store into array

    ROS_INFO("----------------------------");

  }

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);

  ros::spin();

  return 0;
}

