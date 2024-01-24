#include <ros.h>
#include <final_rover/roverServer.h>
#include <std_msgs/Int16.h>
#include <Stepper.h>

#define dir_r1 37
#define dir_l1 51
#define dir_r2 36
#define dir_l2 50
#define speed_r1 10
#define speed_l1 12
#define speed_r2 11
#define speed_l2 13

#define stepsPerRevolution 200

Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

ros::NodeHandle nh;

int left = 0;
int right = 0;

int steps = 0;
bool isPrev = false;

void roverCb(const final_rover::roverServer &msg) {
  left = msg.left;
  right = msg.right;
}

void scienceCb(const std_msgs::Int16 &msg) {
  steps = msg.data;
}

ros::Subscriber<final_rover::roverServer> sub_rover("/rover_server", &roverCb);
ros::Subscriber<std_msgs::Int16> sub_science("/science_server", &scienceCb);

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

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_rover);
  nh.subscribe(sub_science);

  myStepper.setSpeed(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(right > 0)  {
    digitalWrite(dir_r1, HIGH);
    digitalWrite(dir_r2, HIGH);
  }
  else {
    digitalWrite(dir_r1, LOW);
    digitalWrite(dir_r2, LOW);
  }
  analogWrite(speed_r1, (abs(right) < 255)? abs(right) : 255);
  analogWrite(speed_r2, (abs(right) < 255)? abs(right) : 255);

  if(left > 0)  {
    digitalWrite(dir_l1, HIGH);
    digitalWrite(dir_l2, HIGH);
  }
  else {
    digitalWrite(dir_l1, LOW);
    digitalWrite(dir_l2, LOW);
  }
  analogWrite(speed_l1, (abs(left) < 255)? abs(left) : 255);
  analogWrite(speed_l2, (abs(left) < 255)? abs(left) : 255);

  /////////////////////////////////////////////////////////////////////////////////////////////////
  if(isPrev == true && steps == 0) {
    isPrev == false;
  }
  if(isPrev == false && steps != 0){
    myStepper.step(steps);
    delay(500);
    isPrev = true;
  }
  nh.spinOnce();
  delay(1);

}
