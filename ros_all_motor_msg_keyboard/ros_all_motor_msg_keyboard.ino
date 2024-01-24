#include <Servo.h>
#include <ros.h>
#include <ezButton.h>
#include <beginner_tutorials/anglesMsg.h>

#define motor1_speed 9
#define motor2_speed 8
#define motor_base_speed 10
#define motor1_dir 50
#define motor2_dir 52
#define motor_base_dir 48

#define pump_switch 53
#define pump_motor 0

#define CLK_PIN_first 3
#define DT_PIN_first 2
#define SW_PIN_first 4
volatile long counter_first = 0;
volatile unsigned long last_time_first;  // for debouncing
long prev_counter_first;
ezButton button_first(SW_PIN_first);

#define CLK_PIN_second 20
#define DT_PIN_second 21
#define SW_PIN_second 4
volatile long counter_second = 0;
volatile unsigned long last_time_second;  // for debouncing
long prev_counter_second;
ezButton button_second(SW_PIN_second);

ros::NodeHandle nh;

const int speed = 255;
const int speed_base = 35;
const int error = 0.351;
long countsPerRotation_first = 512;
long countsPerRotation_second = 1024;
long countsPerRotation_base = 256;
int servo_1_angle = 0;
int servo_2_angle = 0;

long angle_first = 0;
long angle_second = 0;
long angle_base = 0;

const int referenceAngle_first = 0;
const int referenceAngle_second = 0;
const int referenceAngle_base = 0;

bool isPump = false;

Servo servo_1;
Servo servo_2;

void messageCb(const beginner_tutorials::anglesMsg &msg) {
  while(angle_first != msg.first || angle_second != msg.second )
  {
    if(angle_first != msg.first)
    {
      if(angle_first - msg.first > 0)
      {
        digitalWrite(motor1_dir, LOW);
        analogWrite(motor1_speed, speed);
      }
      else
      {
        digitalWrite(motor1_dir, HIGH);
        analogWrite(motor1_speed, speed);
      }
    }
    if(angle_first == msg.first)
    {
      analogWrite(motor1_speed, 0);
    }

    if(angle_second != msg.second)
    {
      if(angle_second - msg.second > 0)
      {
        digitalWrite(motor2_dir, LOW);
        analogWrite(motor2_speed, speed);
      }
      else
      {
        digitalWrite(motor2_dir, HIGH);
        analogWrite(motor2_speed, speed);
      }
    }
    if(angle_second == msg.second)
    {
      analogWrite(motor2_speed, 0);
    }
  }
  analogWrite(motor1_speed, 0);
  analogWrite(motor2_speed, 0);

  if(msg.base == 'r')
  {
    analogWrite(motor_base_speed, speed_base);
    digitalWrite(motor_base_dir, HIGH);
  }

  else if(msg.base == 'l')
  {
    analogWrite(motor_base_speed, speed_base);
    digitalWrite(motor_base_dir, LOW);
  }

  else if(msg.base == 's')
  {
    analogWrite(motor_base_speed, 0);
  }

  servo_1.write(msg.servo1);
  servo_2.write(msg.servo2);

  if(msg.isPump)
  {
    digitalWrite(pump_switch, HIGH);
  }
  else
  {
    digitalWrite(pump_switch, LOW);   
  }
}

ros::Subscriber<beginner_tutorials::anglesMsg> sub("/input", &messageCb);

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1_speed, OUTPUT);
  pinMode(motor2_speed, OUTPUT);
  pinMode(motor_base_speed, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor_base_dir, OUTPUT);
  pinMode(pump_switch, OUTPUT);
  pinMode(pump_motor, OUTPUT);

  servo_1.attach(7);
  servo_2.attach(6);

  digitalWrite(pump_switch, LOW);
  servo_1.write(servo_1_angle);
  servo_2.write(servo_2_angle);

  pinMode(CLK_PIN_first, INPUT);
  pinMode(DT_PIN_first, INPUT);
  button_first.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_first), ISR_encoderChange_first, RISING);

  pinMode(CLK_PIN_second, INPUT);
  pinMode(DT_PIN_second, INPUT);
  button_second.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_second), ISR_encoderChange_second, RISING);

  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  button_first.loop(); 
  button_second.loop(); 
  delay(1);
}

void ISR_encoderChange_first() {
  if ((millis() - last_time_first) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_first) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    counter_first--;
  } 
  else {
    // the encoder is rotating in clockwise direction => increase the counter
    counter_first++;
  }
  angle_first = referenceAngle_first + (counter_first * 360 / countsPerRotation_first);

  char charArray[20];
  nh.loginfo("First:");
  nh.loginfo(ltoa(angle_first, charArray, 10));

  last_time_first = millis();
}

void ISR_encoderChange_second() {
  if ((millis() - last_time_second) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_second) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    counter_second--;
  } 
  else {
    // the encoder is rotating in clockwise direction => increase the counter
    counter_second++;     
  }
  angle_second = referenceAngle_second + (counter_second * 360 / countsPerRotation_second);

  char charArray[20];
  nh.loginfo("Second:");
  nh.loginfo(ltoa(angle_second, charArray, 10));

  last_time_second = millis();
}

