#include <ros/ros.h>
#include <std_msgs/Int8.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "off_led");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Int8>("led_val", 1000);
	ros::Rate loop_rate(10);
	 std_msgs::Int8 led_off;
	while(ros::ok())
	{
		led_off.data = 0;
		pub.publish(led_off);
    	ros::spin();
    	loop_rate.sleep();
	}
	return 0;
}