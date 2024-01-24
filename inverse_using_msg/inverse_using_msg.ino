#include <Servo.h>
#include <ros.h>
#include <ezButton.h>
#include <beginner_tutorials/anglesMsg.h>


#define motor1_speed 10
#define motor2_speed 11
#define motor_base_speed 4
#define motor1_dir 42
#define motor2_dir 44
#define motor_base_dir 48

#define pump_switch 46
#define pump_motor 46

#define CLK_PIN_first 20
#define DT_PIN_first 21
#define SW_PIN_first 4
volatile long counter_first = 0;
volatile unsigned long last_time_first;  // for debouncing
int prev_counter_first;
ezButton button_first(SW_PIN_first);

#define CLK_PIN_second 19
#define DT_PIN_second 18
#define SW_PIN_second 4
volatile long counter_second = 0;
volatile unsigned long last_time_second;  // for debouncing
int prev_counter_second;
ezButton button_second(SW_PIN_second);

#define CLK_PIN_base 2
#define DT_PIN_base 3
#define SW_PIN_base 4
volatile long counter_base = 0;
volatile unsigned long last_time_base;  // for debouncing
int prev_counter_base;
ezButton button_base(SW_PIN_base);

ros::NodeHandle nh;

const int speed = 255;
const int speed_base = 30;
const int error = 0.351;
long countsPerRotation = 1024;
int servo_1_angle = 0;
int servo_2_angle = 0;

long angle_first = 0;
long angle_second = 0;
long angle_base = 0;

const long referenceAngle_first = 0;
const long referenceAngle_second = 0;
const long referenceAngle_base = 0;

bool isPump = false;

Servo servo_1;
Servo servo_2;

void messageCb(const beginner_tutorials::anglesMsg &msg)
{
  while(angle_first != msg.first)
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
  analogWrite(motor1_speed, 0);

  while(angle_second != msg.second)
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
  analogWrite(motor2_speed, 0);

  while(angle_base != msg.base)
  {
    if(angle_base - msg.base > 0)
    {
      digitalWrite(motor_base_dir, HIGH);
      analogWrite(motor_base_speed, speed_base);
    }
    else
    {
      digitalWrite(motor_base_dir, LOW);
      analogWrite(motor_base_speed, speed_base);
    }
  }
  analogWrite(motor_base_speed, 0);
}

ros::Subscriber<beginner_tutorials::anglesMsg> sub("/angles", &messageCb);

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

  servo_1.attach(5);
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

  pinMode(CLK_PIN_base, INPUT);
  pinMode(DT_PIN_base, INPUT);
  button_base.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_base), ISR_encoderChange_base, RISING);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
}

void loop() {
  // put your main code here, to run repeatedly:
  button_first.loop();
  button_second.loop();
  button_base.loop(); 

  nh.spinOnce();
  nh.subscribe(sub);
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
  angle_first = referenceAngle_first + (counter_first * 360 / countsPerRotation);

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
  angle_second = referenceAngle_second + (counter_second * 360 / countsPerRotation);
  char charArray[10];
  nh.loginfo(ltoa(angle_second, charArray, 10));

  last_time_second = millis();
}

void ISR_encoderChange_base() {
  if ((millis() - last_time_base) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_base) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    counter_base--;
  } 
  else {
    // the encoder is rotating in clockwise direction => increase the counter
    counter_base++;
  }
  angle_base = referenceAngle_base + (counter_base * 360 / countsPerRotation);
  
  last_time_base = millis();
}


