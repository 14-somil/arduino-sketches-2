#include <ros.h>
#include <math.h>
#include <final_rover/armClient.h>
#include <final_rover/encodersFeedback.h>

#define motor1_pwm 9
#define motor2_pwm 8
#define motor_base_pwm 7
#define motor1_dir 45
#define motor2_dir 41  
#define motor_base_dir 49

#define diff1_pwm 13
#define diff2_pwm 11
#define diff1_dir 33
#define diff2_dir 37

#define grip_pwm 5
#define grip_dir 53

#define CLK_PIN_first 3
#define DT_PIN_first 2
volatile long counter_first = 0;
volatile long double last_time_first;  // for debouncing
long prev_counter_first;

#define CLK_PIN_second 21
#define DT_PIN_second 20
volatile long counter_second = 0;
volatile long double last_time_second;  // for debouncing
long prev_counter_second;

#define CLK_PIN_diff1 0
#define DT_PIN_diff1 0
volatile long counter_diff1 = 0;
volatile long double last_time_diff1;  // for debouncing
long prev_counter_diff1;

#define CLK_PIN_diff2 0
#define DT_PIN_diff2 0
volatile long counter_diff2 = 0;
volatile long double last_time_diff2;  // for debouncing
long prev_counter_diff2;

long countsPerRotation_first = 512;
long countsPerRotation_second = 1024;
long countsPerRotation_base = 256;
long countsPerRotation_diff1 = 104;
long countsPerRotation_diff2 = 104;

long double angle_first = 0;
long double angle_second = 0;
long double angle_base = 0;
long double angle_diff1 = 0;
long double angle_diff2 = 0;

long double target_angle_first = 0;
long double target_angle_second = 0;
long double target_angle_base = 0;

const int referenceAngle_first = 0;
const int referenceAngle_second = 0;
const int referenceAngle_base = 0;
const int referenceAngle_diff1 = 0;
const int referenceAngle_diff2 = 0;

int voltage_first=0;
int voltage_second=0;
int voltage_base=0;

double yaw_angle=0;
double pitch_angle=0;
float yaw=0;
float pitch=0;
int gripper=0;
char command = 'c';

ros::NodeHandle nh;

void subCb(const final_rover::armClient &msg) {
  command = msg.command;
  voltage_base = msg.y;
  yaw = msg.yaw;
  pitch = msg.pitch;
  gripper = msg.gripper;
}

final_rover::encodersFeedback angleFeedback;
ros::Publisher pub("/angle_feedback_server", &angleFeedback);
ros::Subscriber<final_rover::armClient> sub("/arm_controller_server", &subCb);
unsigned long last_published=millis();

void calculate_voltage() {
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
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor_base_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor_base_dir, OUTPUT);

  pinMode(diff1_pwm, OUTPUT);
  pinMode(diff2_pwm, OUTPUT);
  pinMode(diff1_dir, OUTPUT);
  pinMode(diff2_dir, OUTPUT);

  pinMode(grip_dir, OUTPUT);
  pinMode(grip_dir, OUTPUT);

  pinMode(CLK_PIN_first, INPUT);
  pinMode(DT_PIN_first, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_first), ISR_encoderChange_first, RISING);

  pinMode(CLK_PIN_second, INPUT);
  pinMode(DT_PIN_second, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_second), ISR_encoderChange_second, RISING);

  pinMode(CLK_PIN_diff1, INPUT);
  pinMode(DT_PIN_diff1, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_diff1), ISR_encoderChange_diff1, RISING);

  pinMode(CLK_PIN_diff2, INPUT);
  pinMode(DT_PIN_diff2, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_diff2), ISR_encoderChange_diff2, RISING);

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
    // angleFeedback.base = angle_base;
    pub.publish(&angleFeedback);
  }

///////////////////////////////////////////////////////////////////////////////////
  yaw_angle = 0.25*(angle_diff1 - angle_diff2);
  pitch_angle = 0.5*(angle_diff1 + angle_diff2);
///////////////////////////////////////////////////////////////////////////////////

  if(command == 'w') {
    digitalWrite(motor1_dir, LOW);
    analogWrite(motor1_pwm, 255);
    analogWrite(motor2_pwm, 0);
  }
  else if(command == 's') {
    digitalWrite(motor1_dir, HIGH);
    analogWrite(motor1_pwm, 255);
    analogWrite(motor2_pwm, 0);
  }
  else if(command == '8') {
    analogWrite(motor1_pwm, 0);
    digitalWrite(motor2_dir, LOW);
    analogWrite(motor2_pwm, 255);
  }
  else if(command == '2') {
    analogWrite(motor1_pwm, 0);
    digitalWrite(motor2_dir, HIGH);
    analogWrite(motor2_pwm, 255);
  }
  else {
    analogWrite(motor1_pwm, 0);
    analogWrite(motor2_pwm, 0);
  }

////////////////////////////////////////////////////////////////////////////////////////////////////
  if(voltage_base > 0) {
    digitalWrite(motor_base_dir, LOW);
  }
  else {
    digitalWrite(motor_base_dir, HIGH);
  }

  analogWrite(motor_base_pwm, abs(voltage_base));

////////////////////////////////////////////////////////////////////////////////////////////////////
  int right=0;
  int left=0;

  right -= ((pitch>0 && pitch_angle<45) || (pitch<0 && pitch_angle>-45))? pitch : 0;
  left += ((pitch>0 && pitch_angle<45) || (pitch<0 && pitch_angle>-45))? pitch : 0;

  right += ((yaw>0 && yaw_angle<180) || (yaw<0 && yaw_angle>-180))? yaw : 0;
  left += ((yaw>0 && yaw_angle<180) || (yaw<0 && yaw_angle>-180))? yaw : 0;

  if(right > 0)  {
    digitalWrite(diff1_dir, HIGH);
  }
  else {
    digitalWrite(diff1_dir, LOW);
  }
  analogWrite(diff1_pwm, (abs(right) < 255)? abs(right) : 255);

  if(left > 0)  {
    digitalWrite(diff2_dir, HIGH);
  }
  else {
    digitalWrite(diff2_dir, LOW);
  }
  analogWrite(diff2_pwm, (abs(left) < 255)? abs(left) : 255);
///////////////////////////////////////////////////////////////////////////////////////////////////////

  // if(gripper != 0) {
  //   if(gripper == 1) {
  //     digitalWrite(grip_dir, LOW);
  //     digitalWrite(grip_pwm, HIGH);
  //   }
    
  //   else {
  //     digitalWrite(grip_dir, HIGH);
  //     digitalWrite(grip_pwm, HIGH);
  //   }
  // }

  // else {
  //   digitalWrite(grip_pwm, LOW);
  //}

  if(gripper == 1) {
    digitalWrite(grip_dir, LOW);
    digitalWrite(grip_pwm, HIGH);
  }

  if(gripper == -1) {
    digitalWrite(grip_dir, HIGH);
    digitalWrite(grip_pwm, HIGH);
  }

  if(gripper == 0) {
    digitalWrite(grip_pwm, LOW);
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////

  nh.spinOnce();
  delay(5);
}

void ISR_encoderChange_first() {
  if ((millis() - last_time_first) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_first) == HIGH) {
    counter_first--;
  } 
  else {
    counter_first++;
  }
  angle_first = referenceAngle_first + (counter_first * 360 / countsPerRotation_first);
  last_time_first = millis();
}

void ISR_encoderChange_second() {
  if ((millis() - last_time_second) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_second) == HIGH) {
    counter_second--;
  } 
  else {
    counter_second++;     
  }
  angle_second = referenceAngle_second + (counter_second * 360 / countsPerRotation_second);
  last_time_second = millis();
}

void ISR_encoderChange_diff1() {
  if ((millis() - last_time_diff1) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_diff1) == HIGH) {
    counter_diff1--;
  } 
  else {
    counter_diff1++;     
  }
  angle_diff1 = referenceAngle_diff1 + (counter_diff1 * 360 / countsPerRotation_diff1);
  last_time_diff1 = millis();
}

void ISR_encoderChange_diff2() {
  if ((millis() - last_time_diff2) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_diff2) == HIGH) {
    counter_diff2--;
  } 
  else {
    counter_diff2++;     
  }
  angle_diff2 = referenceAngle_diff2 + (counter_diff2 * 360 / countsPerRotation_diff2);
  last_time_diff2 = millis();
}

