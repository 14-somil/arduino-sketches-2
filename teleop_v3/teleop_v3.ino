#include <ros.h>
#include <final_rover/RoverMsg.h>

#define dir_r1 37
#define dir_l1 51
#define dir_r2 36
#define dir_l2 50
#define speed_r1 10
#define speed_l1 12
#define speed_r2 11
#define speed_l2 13

int lin_speed = 0;
int rot_speed = 0;

ros::NodeHandle nh;

void messageCb(const final_rover::RoverMsg &msg) {
  lin_speed = msg.x;
  rot_speed = msg.y;
}

ros::Subscriber<final_rover::RoverMsg> sub("/rover_client", &messageCb);

void setup() {
  // put your setup code here, to run once:
  pinMode(dir_r1, OUTPUT);
  pinMode(dir_l1, OUTPUT);
  pinMode(dir_r2, OUTPUT);
  pinMode(dir_l2, OUTPUT);
  pinMode(speed_r1, OUTPUT);
  pinMode(speed_l1, OUTPUT);
  pinMode(speed_r2, OUTPUT);
  pinMode(speed_l2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:

  int right=0;
  int left=0;

  right += lin_speed;
  left += lin_speed;

  right -= rot_speed;
  left += rot_speed;

  if(right > 0)  {
    digitalWrite(dir_r1, LOW);
    digitalWrite(dir_r2, LOW);
  }
  else {
    digitalWrite(dir_r1, HIGH);
    digitalWrite(dir_r2, HIGH);
  }
  analogWrite(speed_r1, (abs(right) < 255)? abs(right) : 255);
  analogWrite(speed_r2, (abs(right) < 255)? abs(right) : 255);

  if(left > 0)  {
    digitalWrite(dir_l1, LOW);
    digitalWrite(dir_l2, LOW);
  }
  else {
    digitalWrite(dir_l1, HIGH);
    digitalWrite(dir_l2, HIGH);
  }
  analogWrite(speed_l1, (abs(left) < 255)? abs(left) : 255);
  analogWrite(speed_l2, (abs(left) < 255)? abs(left) : 255);

  nh.spinOnce();
  delay(1);
}
