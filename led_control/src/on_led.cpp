#include <ros/ros.h>
#include <std_msgs/Int8.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "on_led");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Int8>("led_val", 1000);
	ros::Rate loop_rate(10);
	 std_msgs::Int8 led_on;
	while(ros::ok())
	{
		led_on.data = 1;
		pub.publish(led_on);
    	ros::spinOnce();
    	loop_rate.sleep();
	}
	return 0;
}