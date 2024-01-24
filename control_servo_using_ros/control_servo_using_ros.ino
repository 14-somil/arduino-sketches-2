//ros servo controller

#include <ros.h>
#include <std_msgs/Int64.h>
#include <Servo.h>

Servo myservo;

ros::NodeHandle nh;

void messageCb(const std_msgs::Int64 &toggle_msg) {
  int msg = toggle_msg.data;
  myservo.write(msg);
}

ros::Subscriber<std_msgs::Int64> sub("toggle_servo", &messageCb);

void setup() {
  // put your setup code here, to run once:
  myservo.attach(3);
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
