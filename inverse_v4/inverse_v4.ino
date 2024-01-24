#include <Servo.h>
#include <ros.h>
#include <math.h>
#include <beginner_tutorials/anglesMsg.h>
#include <beginner_tutorials/encodersFeedback.h>

float a1=28.8;
float a2=23.7;
float b1=21.5;
float b2=10.1;

#define motor1_pwm 8
#define motor2_pwm 9
#define motor_base_pwm 0
#define motor1_dir 50
#define motor2_dir 52
#define motor_base_dir 0

#define pot A0

#define pump_switch 53
#define pump_motor 0
bool isPump= false;

Servo servo1;
Servo servo2;
Servo servo3;

#define CLK_PIN_first 3
#define DT_PIN_first 2
volatile long counter_first = 0;
volatile long double last_time_first;  // for debouncing
long prev_counter_first;

#define CLK_PIN_second 20
#define DT_PIN_second 21
volatile long counter_second = 0;
volatile long double last_time_second;  // for debouncing
long prev_counter_second;

long countsPerRotation_first = 512;
long countsPerRotation_second = 1024;
long countsPerRotation_base = 256;

long double angle_first = 0;
long double angle_second = 0;
long double angle_base = 0;

long double angle_first_rad = 0;
long double angle_second_rad = 0;
long double angle_base_rad = 0;

long double target_angle_first = 0;
long double target_angle_second = 0;
long double target_angle_base = 0;

const int referenceAngle_first = 0;
const int referenceAngle_second = 0;
const int referenceAngle_base = 0;

int servo_1_angle = 87;
int servo_2_angle = 60;
int servo_3_angle = 90;

int voltage_first=0;
int voltage_second=0;
int voltage_base=0;

ros::NodeHandle nh;

void subCb(const beginner_tutorials::anglesMsg &msg) {
  target_angle_first = msg.first;
  target_angle_second = msg.second;
  // target_angle_base = msg.base;
  servo_1_angle = msg.servo1;
  servo_2_angle = msg.servo2;
  // servo_3_angle = msg.servo3;
  isPump = msg.isPump;
}

beginner_tutorials::encodersFeedback angleFeedback;
ros::Publisher pub("/angleFeedback", &angleFeedback);
ros::Subscriber<beginner_tutorials::anglesMsg> sub("/input", &subCb);
unsigned long last_published;

void calculate_voltage() {
  // angle_first_rad = (90- angle_first) *PI/180;
  // angle_second_rad = angle_second *PI/180;

  // double d_first = (target_angle_first - angle_first)*PI/180;
  // double d_second = (target_angle_second - angle_second)*PI/180;

  // double d_x = -(a1*a2*sin(angle_first_rad)*d_first)/sqrt( pow((a1 + a2*cos(angle_first_rad)),2) + pow(a2*sin(angle_first_rad), 2));
  // double d_y = (b1*b2*cos(angle_second_rad)*d_second)/sqrt( pow((b1 + b2*sin(angle_second_rad)),2) + pow(b2*cos(angle_second_rad), 2));

  // d_y = 10*d_y/7;

  // if(d_x>d_y)
  // {
  //   double ratio = d_y/d_x;
  //   voltage_first = 255;
  //   voltage_second = abs(255*ratio);
  // }

  // else
  // {
  //   double ratio = d_x/d_y;
  //   voltage_first = abs(255*ratio);
  //   voltage_second = 255;
  // }
  float k= 50;
  voltage_first = k * abs(target_angle_first - angle_first);
  voltage_second = k * abs(target_angle_second - angle_second);

  if(voltage_first>255) {
    voltage_first = 255;
  }
  else if(voltage_first<0) {
    voltage_first = 0;
  }

  if(voltage_second>255) {
    voltage_second = 255;
  }
  else if(voltage_second<0) {
    voltage_second = 0;
  }

}

void setup() {
  // put your setup code here, to run once:
  pinMode(pot, INPUT);

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor_base_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor_base_dir, OUTPUT);

  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(6);

  digitalWrite(pump_switch, LOW);
  servo1.write(servo_1_angle);
  servo2.write(servo_2_angle);
  servo3.write(servo_3_angle);

  pinMode(CLK_PIN_first, INPUT);
  pinMode(DT_PIN_first, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_first), ISR_encoderChange_first, RISING);

  pinMode(CLK_PIN_second, INPUT);
  pinMode(DT_PIN_second, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_second), ISR_encoderChange_second, RISING);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  last_published = millis();

}

void loop() {
  // put your main code here, to run repeatedly:

  if(millis()-last_published > 10) {
    angleFeedback.first = angle_first;
    angleFeedback.second = angle_second;
    angleFeedback.base = angle_base;
    pub.publish(&angleFeedback);
  }
/////////////////////////////////////////////////////////////////////////////////
  int pot_input = analogRead(pot);                                             //
  angle_base = map(pot_input, 0, 1024, -1800, 1800);                           //
/////////////////////////////////////////////////////////////////////////////////
  calculate_voltage();

  if((int)target_angle_first != (int)angle_first) {
    if(target_angle_first > angle_first)
      digitalWrite(motor1_dir, HIGH);
    else
      digitalWrite(motor1_dir, LOW);
    
    analogWrite(motor1_pwm, voltage_first);
  }
  else {
    analogWrite(motor1_pwm, 0);
  }

  if((int)target_angle_second != (int)angle_second) {
    if(target_angle_second > angle_second)
      digitalWrite(motor2_dir, HIGH);
    else
      digitalWrite(motor2_dir, LOW);
    
    analogWrite(motor2_pwm, voltage_second);
  }
  else {
    analogWrite(motor2_pwm, 0);
  }



  nh.spinOnce();
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

  // char charArray[20];
  // nh.loginfo("First:");
  // nh.loginfo(ltoa(angle_first, charArray, 10));

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

  // char charArray[20];
  // nh.loginfo("Second:");
  // nh.loginfo(ltoa(angle_second, charArray, 10));

  last_time_second = millis();
}


