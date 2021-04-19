#include <ros/ros.h>
#include <signal.h>

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()

  ROS_INFO("Giving Stop Signal");
  //system("rosnode kill turtlebot3_teleop_keyboard");
  //system("rosnode list | grep -v rosout | xargs rosnode kill");
  system("pkill roslaunch");
  //system("rosnode kill shutdown_node");

  //system("ps -ef | sed -n '/led_listener/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
  ros::requestShutdown();
  //ros::shutdown();
  //rosnode list | grep -v rosout | xargs rosnode kill
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_node_name", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);
  
  ros::spin();
  return 0;
}