#define motor1_speed 3
#define motor2_speed 0
#define motor3_speed 0
#define motor1_dir 4
#define motor2_dir 0
#define motor3_dir 0

#define pump_switch 8

const int speed=50;
bool dataIn=false;
char input;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1_speed, OUTPUT);
  pinMode(motor2_speed, OUTPUT);
  pinMode(motor3_speed, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor3_dir, OUTPUT);
  pinMode(pump_switch, OUTPUT);

  digitalWrite(pump_switch, LOW);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0);
  {
    input = Serial.read();
    dataIn = true;
  }

  if(dataIn == true)
  {
    switch(input)
    {
      case 'q':
      {
        analogWrite(motor1_speed, speed);
        digitalWrite(motor1_dir, LOW);
        delay(1000);

        break;
      }

      case 'a':
      {
        analogWrite(motor1_speed, speed);
        digitalWrite(motor1_dir, HIGH);
        delay(1000);

        break;
      }

      case 'w':
      {
        analogWrite(motor2_speed, speed);
        digitalWrite(motor2_dir, LOW);
        delay(1000);

        break;
      }

      case 's':
      {
        analogWrite(motor2_speed, speed);
        digitalWrite(motor2_dir, HIGH);
        delay(1000);

        break;
      }

      case 'e':
      {
        analogWrite(motor3_speed, speed);
        digitalWrite(motor3_dir, LOW);
        delay(1000);

        break;
      }

      case 'd':
      {
        analogWrite(motor3_speed, speed);
        digitalWrite(motor3_dir, HIGH);
        delay(1000);

        break;
      }

      case 'o':
      {
        digitalWrite(pump_switch, HIGH);
        
        break;
      }

      case 'l':
      {
        digitalWrite(pump_switch, LOW);
        
        break;
      }

      default:
      {
        analogWrite(motor1_speed, 0);
        analogWrite(motor2_speed, 0);
        analogWrite(motor3_speed, 0);

        break;
      }
    }

    analogWrite(motor1_speed, 0);
    analogWrite(motor2_speed, 0);
    analogWrite(motor3_speed, 0);

    dataIn=false;
  }

}
