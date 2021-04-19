#include <ros/ros.h>
#include <std_msgs/Int8.h>

std_msgs::Int8 val;
void messageCallback(const std_msgs::Int8& led_msg)
{
  ROS_INFO("Subscribe from terminal - ros_tutorial_msg topic = [%d]", led_msg);
  val = led_msg;
  ros::NodeHandle nh2;
  ros::Publisher Led_stt = nh2.advertise<std_msgs::Int8>("led_val", 1000, true);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    Led_stt.publish(val);
    ROS_INFO("Publish to arduino = [%d]", val);
    ros::spin();
    loop_rate.sleep();
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "led_listener");// Initializes Node Name
  ros::NodeHandle nh;// Node handle declaration for communication with ROS system

  ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 1000, &messageCallback);

  ros::spin();

  return 0;
}

