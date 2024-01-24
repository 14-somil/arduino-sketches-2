#include <ros.h>
#include <ezButton.h>
#include <beginner_tutorials/encodersFeedback.h>
#include <beginner_tutorials/speedMsg.h>

#define motor1_pwm 8
#define motor2_pwm 9
#define motor_base_pwm 0
#define motor1_dir 50
#define motor2_dir 52
#define motor_base_dir 0

#define CLK_PIN_first 3
#define DT_PIN_first 2
#define SW_PIN_first 4
volatile long counter_first = 0;
volatile long double last_time_first;  // for debouncing
long prev_counter_first;
ezButton button_first(SW_PIN_first);

#define CLK_PIN_second 20
#define DT_PIN_second 21
#define SW_PIN_second 4
volatile long counter_second = 0;
volatile long double last_time_second;  // for debouncing
long prev_counter_second;
ezButton button_second(SW_PIN_second);

long double countsPerRotation_first = 512;
long double countsPerRotation_second = 1024;
long double countsPerRotation_base = 256;

long double angle_first = 0;
long double angle_second = 0;
long double angle_base = 0;

const double referenceAngle_first = 0;
const double referenceAngle_second = 0;
const double referenceAngle_base = 0;

double speed_cur_first=0;
double speed_des_first=0;
double k_first = 0.5;
double voltage_first=0;

double speed_cur_second=0;
double speed_des_second=0;
double k_second = 0.5;
double voltage_second=0;

ros::NodeHandle nh;

void speedCallback(const beginner_tutorials::speedMsg &msg)
{
  speed_des_first = msg.first;
  speed_des_second = msg.second;

  char buffer[20]; // Adjust the buffer size based on your needs
  dtostrf(speed_des_first, 5, 2, buffer); 
  nh.loginfo(buffer);
}

beginner_tutorials::encodersFeedback angleMsg;
ros::Publisher pub("/angleFeed", &angleMsg);
ros::Subscriber<beginner_tutorials::speedMsg> sub("/speedFeed", &speedCallback);
float lastPublished = millis();

void setup() {
  // put your setup code here, to run once :

  // Serial.begin(9600);

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor_base_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor_base_dir, OUTPUT);

  pinMode(CLK_PIN_first, INPUT);
  pinMode(DT_PIN_first, INPUT);
  button_first.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_first), ISR_encoderChange_first, RISING);

  pinMode(CLK_PIN_second, INPUT);
  pinMode(DT_PIN_second, INPUT);
  button_second.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_second), ISR_encoderChange_second, RISING);
 
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop() {
  // put your main code here, to run repeatedly:

  button_first.loop(); 
  button_second.loop();

  //control first joint
  if(millis()-last_time_first > 250)
  {
    speed_cur_first=0;
  } 

  if(speed_des_first>0)
  {
    digitalWrite(motor1_dir, HIGH); 
  }
  else
  {
    digitalWrite(motor1_dir, LOW); 
  }

  voltage_first -= k_first*(abs(speed_cur_first)- abs(speed_des_first));
  if(voltage_first>255)
    voltage_first=255;
  if(voltage_first<0)
    voltage_first=0;
  analogWrite(motor1_pwm, voltage_first);


  //control second joint
  if(millis()-last_time_second > 250)
  {
    speed_cur_second=0;
  } 

  if(speed_des_second>0)
  {
    digitalWrite(motor2_dir, HIGH); 
  }
  else
  {
    digitalWrite(motor2_dir, LOW); 
  }

  voltage_second -= k_second*(abs(speed_cur_second)- abs(speed_des_second));
  if(voltage_second>255)
    voltage_second=255;
  if(voltage_second<0)
    voltage_second=0;
  analogWrite(motor2_pwm, voltage_second);

  delay(8);


  //publisher
  if(millis()-lastPublished > 10)
  {
    angleMsg.first = angle_first;
    angleMsg.second = angle_second;
    angleMsg.base = angle_base;
    pub.publish(&angleMsg);
    lastPublished = millis();
  }

  nh.spinOnce();
}

void ISR_encoderChange_first() {
  if ((millis() - last_time_first) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_first) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    counter_first--;
    speed_cur_first = -1000*(360 / countsPerRotation_first)/(millis()-last_time_first);
  } 
  else {
    // the encoder is rotating in clockwise direction => increase the counter
    counter_first++;
    speed_cur_first = 1000*(360 / countsPerRotation_first)/(millis()-last_time_first);
  }
  angle_first = referenceAngle_first + (counter_first * 360 / countsPerRotation_first);
  
  last_time_first = millis();
}

void ISR_encoderChange_second() {
  if ((millis() - last_time_second) < 50)  // debounce time is 50ms
    return;


  if (digitalRead(DT_PIN_second) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    counter_second--;
    speed_cur_second = -1000*(360 / countsPerRotation_second)/(millis()-last_time_second);
  } 
  else {
    // the encoder is rotating in clockwise direction => increase the counter
    counter_second++;
    speed_cur_second = 1000*(360 / countsPerRotation_second)/(millis()-last_time_second);
  }
  angle_second = referenceAngle_second + (counter_second * 360 / countsPerRotation_second);
  
  last_time_second = millis();
}

