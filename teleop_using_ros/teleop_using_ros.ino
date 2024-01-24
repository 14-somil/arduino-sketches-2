#include <ros.h>
#include <std_msgs/Char.h>

#define dir_1 7
#define dir_2 8
#define dir_3 12
#define dir_4 13
#define speed_1 6
#define speed_2 9
#define speed_3 5
#define speed_4 3

const int speed = 100;

ros::NodeHandle nh;

char inputChar;

void messageCb(const std_msgs::Char &toggle_msg) {
  inputChar = toggle_msg.data;
  switch (inputChar) {
    case 'w':
      {

        digitalWrite(dir_1, LOW);
        digitalWrite(dir_2, HIGH);
        digitalWrite(dir_3, LOW);
        digitalWrite(dir_4, HIGH);

        analogWrite(speed_1, speed);
        analogWrite(speed_2, speed);
        analogWrite(speed_3, speed);
        analogWrite(speed_4, speed);
        delay(1000);

        break;
      }
    case 's':
      {

        digitalWrite(dir_1, HIGH);
        digitalWrite(dir_2, LOW);
        digitalWrite(dir_3, HIGH);
        digitalWrite(dir_4, LOW);

        analogWrite(speed_1, speed);
        analogWrite(speed_2, speed);
        analogWrite(speed_3, speed);
        analogWrite(speed_4, speed);
        delay(1000);

        break;
      }
    case 'a':
      {

        digitalWrite(dir_1, LOW);
        digitalWrite(dir_2, LOW);
        digitalWrite(dir_3, HIGH);
        digitalWrite(dir_4, HIGH);

        analogWrite(speed_1, 255);
        analogWrite(speed_2, 255);
        analogWrite(speed_3, 255);
        analogWrite(speed_4, 255);
        delay(1000);

        break;
      }
    case 'd':
      {

        digitalWrite(dir_1, HIGH);
        digitalWrite(dir_2, HIGH);
        digitalWrite(dir_3, LOW);
        digitalWrite(dir_4, LOW);

        analogWrite(speed_1, 255);
        analogWrite(speed_2, 255);
        analogWrite(speed_3, 255);
        analogWrite(speed_4, 255);
        delay(1000);

        break;
      }
    default:
      {
        // Serial.println("invalid");

        analogWrite(speed_1, 0);
        analogWrite(speed_2, 0);
        analogWrite(speed_3, 0);
        analogWrite(speed_4, 0);

        break;
      }
  }

  analogWrite(speed_1, 0);
  analogWrite(speed_2, 0);
  analogWrite(speed_3, 0);
  analogWrite(speed_4, 0);
}

ros::Subscriber<std_msgs::Char> sub("/cmd_vel", &messageCb);

void setup() {
  // put your setup code here, to run once:
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(dir_3, OUTPUT);
  pinMode(dir_4, OUTPUT);
  pinMode(speed_1, OUTPUT);
  pinMode(speed_2, OUTPUT);
  pinMode(speed_3, OUTPUT);
  pinMode(speed_4, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
