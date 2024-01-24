#include <ezButton.h>

#define motor1_speed 8
#define motor2_speed 9
#define motor_base_speed 10
#define motor1_dir 50
#define motor2_dir 52
#define motor_base_dir 48

#define pump_switch 53
#define pump_motor 0

#define CLK_PIN_first 2
#define DT_PIN_first 3
#define SW_PIN_first 4
volatile long counter_first = 0;
volatile unsigned long last_time_first;  // for debouncing
long prev_counter_first;
ezButton button_first(SW_PIN_first);

#define CLK_PIN_second 21
#define DT_PIN_second 20
#define SW_PIN_second 4
volatile long counter_second = 0;
volatile unsigned long last_time_second;  // for debouncing
long prev_counter_second;
ezButton button_second(SW_PIN_second);

const int speed = 255;
const int speed_base = 35;
const int error = 0.351;
long double countsPerRotation_first = 512;
long double countsPerRotation_second = 1024;
long double countsPerRotation_base = 256;
int servo_1_angle = 0;
int servo_2_angle = 0;

long double angle_first = 0;
long double angle_second = 0;
long double angle_base = 0;

const float referenceAngle_first = 90;
const float referenceAngle_second = 90;
const float referenceAngle_base = 0;

double a=54.1;
double b=52.5;
double a1=28.7;
double a2=24;
double b1=22;
double b2=10;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(motor1_speed, OUTPUT);
  pinMode(motor2_speed, OUTPUT);
  pinMode(motor_base_speed, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor_base_dir, OUTPUT);
  pinMode(pump_switch, OUTPUT);
  pinMode(pump_motor, OUTPUT);

  digitalWrite(pump_switch, LOW);

  pinMode(CLK_PIN_first, INPUT);
  pinMode(DT_PIN_first, INPUT);
  button_first.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_first), ISR_encoderChange_first, RISING);

  pinMode(CLK_PIN_second, INPUT);
  pinMode(DT_PIN_second, INPUT);
  button_second.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_second), ISR_encoderChange_second, RISING);

  delay(8000);
}

void loop() {
  // put your main code here, to run repeatedly:
  button_first.loop(); 
  button_second.loop(); 

  float angle_first_rad = angle_first * PI /180;
  float angle_second_rad = angle_second * PI /180;

  float speed_first = -1/(a*(sin(angle_first_rad) + cos(angle_first_rad)* tan(angle_second_rad)));
  float speed_second = (a*cos(angle_first_rad) + b*cos(angle_first_rad + angle_second_rad))/(a*b*cos(angle_second_rad));

  double speed_x = (-(a1+a2*cos(angle_first_rad))*a2*sin(angle_first_rad) + a2*a2*sin(angle_first_rad)*cos(angle_first_rad))*speed_first/sqrt(pow(a1+a2*cos(angle_first_rad),2) + pow(a2*sin(angle_first_rad),2));
  double speed_y = (-(b1+b2*cos(angle_second_rad))*b2*sin(angle_second_rad) + b2*b2*sin(angle_second_rad)*cos(angle_second_rad))*speed_second/sqrt(pow(b1+b2*cos(angle_second_rad),2) + pow(b2*sin(angle_second_rad),2));

  speed_y = speed_y/0.7;

  if(speed_x>0)
  {
    digitalWrite(motor1_dir,HIGH);
  } 
  else
  {
    digitalWrite(motor1_dir, LOW);
  }

  if(speed_y>0)
  {
    digitalWrite(motor2_dir,LOW);
  } 
  else
  {
    digitalWrite(motor2_dir, HIGH);
  }

  float ratio = speed_x/speed_y;
  if(abs(ratio)>1)
  {
    int temp = abs(255/ratio);
    analogWrite(motor1_speed, 255);
    analogWrite(motor2_speed, temp);
  }
  else
  {
    int temp = abs(255*ratio);
    analogWrite(motor1_speed, temp);
    analogWrite(motor2_speed, 255);
  }

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

  Serial.print("First: ");
  Serial.println(ltoa(angle_first, charArray,10));

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
  char charArray[20];

  Serial.print("Second: ");
  Serial.println(ltoa(angle_second, charArray,10));

  // nh.loginfo("Second:");
  // nh.loginfo(ltoa(angle_second, charArray, 10));

  last_time_second = millis();
}

