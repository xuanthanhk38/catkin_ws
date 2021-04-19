// C library headers
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/ioctl.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "turtlebot3_msgs/VersionInfo.h"
#include "turtlebot3_msgs/SensorState.h"

/**********************Define publish/subscribe cycle**********************/

#define FIRMWARE_VER "1.2.6"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    10   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 
#define TICK2RAD                               0.000116296296 //Default tick2rad waffle or waffle pi
#define WHEEL_RADIUS                           0.0725 //Default waffle or waffle pi
#define WHEEL_SEPARATION                       0.292 //Default waffle or waffle pi

/**********************Define publish/subscribe cycle**********************/

/**********************Processing class**********************/
class SerialPort //Serial class
{
    private:        
        termios tty;    
        char buffer [256];   
        int count = 0;
        int num_bytes;      
    public:      
        int serial_port;
        bool initial(const char* portname) //Initial serial port
        {   
            bool in_return = false;
            tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
            tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
            tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
            tty.c_cflag |= CS8; // 8 bits per byte (most common)
            tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

            tty.c_lflag &= ~ICANON;
            tty.c_lflag &= ~ECHO; // Disable echo
            tty.c_lflag &= ~ECHOE; // Disable erasure
            tty.c_lflag &= ~ECHONL; // Disable new-line echo
            tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
            tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

            tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
            cfsetspeed(&tty, B115200);  
            tty.c_cc[VTIME] = 3;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
            tty.c_cc[VMIN] = 0;
            serial_port = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
            

            tcflush( serial_port, TCIFLUSH );
            if ( tcsetattr ( serial_port, TCSANOW, &tty ) != 0) {
                std::cout << "Error " << errno << " from tcsetattr" << std::endl;
                in_return = false;
            }
            else in_return = true;
            
            return in_return;       
        }               

        bool available()
        {
            int nread = 0;
            ioctl(serial_port, FIONREAD, &nread);
            if (nread > 0) {
                return true;
            }
            else return false;
        }   

        std::string readString()
        {
            memset(&buffer, '\0', sizeof(buffer));
            int num_bytes = read(serial_port, &buffer, sizeof(buffer));
            return std::string(buffer);
        }

        std::string readStringUntilNewline()
        {
            char c[1];
            std::string in_return = "";
            read(serial_port, &c, 1);
            if(c[0] != '\n')
            {
                buffer[count] = c[0];
                count++;
                return in_return;
            }
            else  
            {
                count = 0;
                in_return = std::string(buffer);
                memset(&buffer, '\0', sizeof(buffer));
                return in_return;
            }       
        }

        void Close()
        {
            close(serial_port);
        }

        void Clear()
        {
            ioctl(serial_port, TCFLSH, 2);
        }
};

bool isOnlyDouble(const char* str)
{
    char* endptr = 0;
    strtod(str, &endptr);

    if(*endptr != '\0' || endptr == str)
        return false;
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**********************Advertise pub / sub**********************/
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
  ros::Publisher sensor_state_pub = n.advertise<turtlebot3_msgs::SensorState>("sensor_state", 20);
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 20);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 20);
  ros::Publisher version_info_pub = n.advertise<turtlebot3_msgs::VersionInfo>("firmware_version", 20);
  /**********************Advertise pub / sub**********************/

  /**********************ROS time and frequency**********************/
  ros::Time current_time, lastOdomPublishFrequency, lastVersionPublishFrequency, lastImuPublishFrequency, last_time;
  current_time = ros::Time::now();
  lastOdomPublishFrequency = ros::Time::now();
  lastVersionPublishFrequency = ros::Time::now();
  lastImuPublishFrequency = ros::Time::now();
  
  ros::Duration OdomPublishFrequency(1 / DRIVE_INFORMATION_PUBLISH_FREQUENCY);
  ros::Duration VersionPublishFrequency(1 / VERSION_INFORMATION_PUBLISH_FREQUENCY);
  ros::Duration ImuPublishFrequency(1 / IMU_PUBLISH_FREQUENCY);
  /**********************ROS time and frequency**********************/

  tf::TransformBroadcaster odom_broadcaster;

  /**********************Init serial port**********************/
  SerialPort serial;
  serial.initial("/dev/ttyACM0");
  serial.Clear();
  /**********************Init serial port**********************/

  /**********************Debug data**********************/
  long encoder_left_tick = 0;
  long encoder_right_tick = 0;

  long previous_encoder_left_tick = 0;
  long previous_encoder_right_tick = 0;

  double last_theta = 0;
  double odom_pose[3];
  double odom_vel[3];
  /**********************Debug data**********************/

  while (ros::ok())
  {
    if(serial.available())
    {   
        std::string data[13];
        //std::string in_return = serial.readStringUntilNewline();
        //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
        //write(serial_port, msg, sizeof(msg));
        serial.write_sr();

    }

    ros::spinOnce();
  }

  serial.Close();
}


