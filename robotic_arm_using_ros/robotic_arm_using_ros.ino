#include <Servo.h>
#include <ros.h>
#include <std_msgs/Char.h>
#include <ezButton.h>

#define motor1_speed 8
#define motor2_speed 9
#define motor_base_speed 10
#define motor1_dir 52
#define motor2_dir 50
#define motor_base_dir 48

#define pump_switch 53
#define pump_motor 0

#define CLK_PIN_base 18
#define DT_PIN_base 19
#define SW_PIN_base 4
volatile long counter_base = 0;
volatile unsigned long last_time_base;  // for debouncing
long prev_counter_base;
ezButton button_base(SW_PIN_base);
long countsPerRotation_base = 1024;
long angle_base = 0;
const int referenceAngle_base = 0;

ros::NodeHandle nh;

const int speed = 255;
const int speed_base = 35;
char inputChar;
int servo_1_angle = 0;
int servo_2_angle = 0;

bool isPump = false;

Servo servo_1;
Servo servo_2;

void messageCb(const std_msgs::Char &toggle_msg) {
  inputChar = toggle_msg.data;
  switch (inputChar) {
    case 'q':
      {

        analogWrite(motor1_speed, speed);
        digitalWrite(motor1_dir, LOW);

        break;
      }

    case 'a':
      {

        analogWrite(motor1_speed, speed);
        digitalWrite(motor1_dir, HIGH);

        break;
      }

    case 'e':
      {

        analogWrite(motor2_speed, speed);
        digitalWrite(motor2_dir, LOW);

        break;
      }

    case 'd':
      {

        analogWrite(motor2_speed, speed);
        digitalWrite(motor2_dir, HIGH);

        break;
      }

    case 'n':
      {

        analogWrite(motor_base_speed, speed_base);
        digitalWrite(motor_base_dir, LOW);

        break;
      }

    case 'm':
      {

        analogWrite(motor_base_speed, speed_base);
        digitalWrite(motor_base_dir, HIGH);

        break;
      }

    case 'k':
      {

        if (servo_1_angle - 5 >= 0) {
          servo_1_angle -= 5;
          servo_1.write(servo_1_angle);
        }

        break;
      }

    case 'j':
      {

        if (servo_1_angle + 5 <= 180) {
          servo_1_angle += 5;
          servo_1.write(servo_1_angle);
        }

        break;
      }

    case 'i':
      {

        if (servo_2_angle - 5 >= 0) {
          servo_2_angle -= 5;
          servo_2.write(servo_2_angle);
        }

        break;
      }

    case 'o':
      {

        if (servo_2_angle + 5 <= 180) {
          servo_2_angle += 5;
          servo_2.write(servo_2_angle);
        }

        break;
      }

    case 'z':
      {
        if (isPump == false) {
          digitalWrite(pump_switch, HIGH);
          digitalWrite(pump_motor, HIGH);
          isPump=true;
        } else {
          digitalWrite(pump_switch, LOW);
          digitalWrite(pump_motor, LOW);
          isPump=false;
        }

        break;
      }

    case 'c':
      {
        analogWrite(motor1_speed, 0);
        analogWrite(motor2_speed, 0);
        analogWrite(motor_base_speed, 0);

        break;
      }

    case 'x':
      {
        analogWrite(motor1_speed, 0);
        analogWrite(motor2_speed, 0);
        analogWrite(motor_base_speed, 0);
        digitalWrite(pump_switch, LOW);
        digitalWrite(pump_motor, LOW);

        break;
      }

    default:
      {
        analogWrite(motor1_speed, 0);
        analogWrite(motor2_speed, 0);
        analogWrite(motor_base_speed, 0);

        break;
      }
  }
}

ros::Subscriber<std_msgs::Char> sub("/input", &messageCb);

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

  pinMode(CLK_PIN_base, INPUT_PULLUP);
  pinMode(DT_PIN_base, INPUT_PULLUP);
  button_base.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_base), ISR_encoderChange_base, RISING);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  button_base.loop(); 
  delay(1);
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
  angle_base = referenceAngle_base + (counter_base * 360 / countsPerRotation_base);
  char charArray[20];
  nh.loginfo("Base:");
  nh.loginfo(ltoa(angle_base, charArray, 10));
  
  last_time_base = millis();
}