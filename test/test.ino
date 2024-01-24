#define dir_1 6
#define dir_2 7
// #define dir_3 12
// #define dir_4 13
#define speed_1 10
#define speed_2 11
// #define speed_3 5
// #define speed_4 3

const int speed = 100;
const int rot_speed = 150;

char inputChar;

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

  Serial.begin(9600);
}

void loop(){
  if(Serial.available()>0)
  {
    inputChar = Serial.read();
    if(inputChar == 'w')
    {
      digitalWrite(dir_1, HIGH);
      digitalWrite(dir_2, HIGH);
      // digitalWrite(dir_3, LOW);
      // digitalWrite(dir_4, HIGH);

      analogWrite(speed_1, speed);
      analogWrite(speed_2, speed);
      delay(5000);
    }

    else if(inputChar == 's')
    {
      digitalWrite(dir_1, LOW);
      digitalWrite(dir_2, LOW);
      // digitalWrite(dir_3, LOW);
      // digitalWrite(dir_4, HIGH);

      analogWrite(speed_1, speed);
      analogWrite(speed_2, speed);
      delay(5000);
    }
  }

  analogWrite(speed_1, 0);
  analogWrite(speed_2, 0);

}
