#include <ezButton.h>

#define motor1_pwm 8
#define motor1_dir 50

#define CLK_PIN_first 2
#define DT_PIN_first 3
#define SW_PIN_first 4
volatile long counter_first = 0;
volatile long double last_time_first;  // for debouncing
long prev_counter_first;
ezButton button_first(SW_PIN_first);

long double countsPerRotation_first = 512;
long double angle_first = 0;
const double referenceAngle_first = 90;

double speed_cur_first=0;
double speed_des_first=-2;
double k_first = 0.8;
double voltage_first=0;

double error;

long start;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);

  pinMode(CLK_PIN_first, INPUT);
  pinMode(DT_PIN_first, INPUT);
  button_first.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN_first), ISR_encoderChange_first, RISING);
  start=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  button_first.loop(); 

  // if(millis() - start >5000)
  // {
  //   speed_des_first = 0;
  //   Serial.println("check");
  // }

  // if(millis()-last_time_first > 500)
  // {
  //   speed_cur_first=0;
  // } 

  if(speed_des_first>0)
  {
    digitalWrite(motor1_dir, LOW); 
  }
  else
  {
    digitalWrite(motor1_dir, HIGH); 
  }

  error = abs(speed_cur_first)- abs(speed_des_first) ;
  voltage_first -= 0.5* error;
  if(voltage_first>255)
    voltage_first=255;
  if(voltage_first<0)
    voltage_first=0;
  analogWrite(motor1_pwm, voltage_first);

  Serial.print(voltage_first);
  Serial.print(": ");
  Serial.println(speed_cur_first);
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

