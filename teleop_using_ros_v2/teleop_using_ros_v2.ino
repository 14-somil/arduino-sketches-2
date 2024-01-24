#include <ros.h>
#include <std_msgs/Char.h>

#define dir_1 50
#define dir_2 48
// #define dir_3 12
// #define dir_4 13
#define speed_1 4
#define speed_2 3
// #define speed_3 5
// #define speed_4 3

const int speed = 100;
const int rot_speed = 150;

ros::NodeHandle nh;

char inputChar;

void messageCb(const std_msgs::Char &toggle_msg) {
  inputChar = toggle_msg.data;
  switch (inputChar) {
    case 's':
      {

        digitalWrite(dir_1, HIGH);
        digitalWrite(dir_2, HIGH);
        // digitalWrite(dir_3, LOW);
        // digitalWrite(dir_4, HIGH);

        analogWrite(speed_1, speed);
        analogWrite(speed_2, speed);
        // analogWrite(speed_3, speed);
        // analogWrite(speed_4, speed);

        break;
      }
    case 'w':
      {

        
        digitalWrite(dir_1, LOW);
        digitalWrite(dir_2, LOW);
        // digitalWrite(dir_3, HIGH);
        // digitalWrite(dir_4, LOW);

        analogWrite(speed_1, speed);
        analogWrite(speed_2, speed);
        // analogWrite(speed_3, speed);
        // analogWrite(speed_4, speed);

        break;
      }
    case 'd':
      {

        digitalWrite(dir_1, HIGH);
        digitalWrite(dir_2, LOW);
        // digitalWrite(dir_3, HIGH);
        // digitalWrite(dir_4, HIGH);

        analogWrite(speed_1, rot_speed);
        analogWrite(speed_2, rot_speed);
        // analogWrite(speed_3, rot_speed);
        // analogWrite(speed_4, rot_speed);

        break;
      }
    case 'a':
      {

        
        digitalWrite(dir_1, LOW);
        digitalWrite(dir_2, HIGH);
        // digitalWrite(dir_3, LOW);
        // digitalWrite(dir_4, LOW);

        analogWrite(speed_1, rot_speed);
        analogWrite(speed_2, rot_speed);
        // analogWrite(speed_3, rot_speed);
        // analogWrite(speed_4, rot_speed);

        break;
      }
    case 'k':
      {
        analogWrite(speed_1, 0);
        analogWrite(speed_2, 0);
        // analogWrite(speed_3, 0);
        // analogWrite(speed_4, 0);

        break;
      }
    
    case 'x':
      {
        analogWrite(speed_1, 0);
        analogWrite(speed_2, 0);
        // analogWrite(speed_3, 0);
        // analogWrite(speed_4, 0);

        break;
      }
    
    default:
      {
        // Serial.println("invalid");

        analogWrite(speed_1, 0);
        analogWrite(speed_2, 0);
        // analogWrite(speed_3, 0);
        // analogWrite(speed_4, 0);

        break;
      }
  }

  
}

ros::Subscriber<std_msgs::Char> sub("/cmd_vel", &messageCb);

void setup() {
  // put your setup code here, to run once:
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  // pinMode(dir_3, OUTPUT);
  // pinMode(dir_4, OUTPUT);
  pinMode(speed_1, OUTPUT);
  pinMode(speed_2, OUTPUT);
  // pinMode(speed_3, OUTPUT);
  // pinMode(speed_4, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
