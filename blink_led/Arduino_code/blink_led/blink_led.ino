#include <ros.h>
#include <std_msgs/Int8.h>

#define led 13

ros::NodeHandle nh;

int led_val;

void messageCallback(const std_msgs::Int8& led_msg)
{
  if(led_msg.data == 1)
  {
    digitalWrite(led, HIGH);
  }
  else
  {
    digitalWrite(led, LOW);
  }
}

ros::Subscriber<std_msgs::Int8> sub("led_val", messageCallback);
void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(2);

}
