#define dir_1 2
#define dir_2 4
#define dir_3 11
#define dir_4 8
#define speed_1 3
#define speed_2 5
#define speed_3 9
#define speed_4 6

const int speed = 75;




void setup() {

  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(dir_3, OUTPUT);
  pinMode(dir_4, OUTPUT);
  pinMode(speed_1, OUTPUT);
  pinMode(speed_2, OUTPUT);
  pinMode(speed_3, OUTPUT);
  pinMode(speed_4, OUTPUT);
}

void loop() {
  digitalWrite(dir_1, LOW);
  digitalWrite(dir_2, HIGH);
  digitalWrite(dir_3, HIGH);
  digitalWrite(dir_4, HIGH);



  analogWrite(speed_1, speed);
  analogWrite(speed_2, speed);
  analogWrite(speed_3, speed);
  analogWrite(speed_4, speed);
  delay(2000);



  digitalWrite(dir_1, HIGH);
  digitalWrite(dir_2, LOW);
  digitalWrite(dir_3, LOW);
  digitalWrite(dir_4, LOW);

  analogWrite(speed_1, speed);
  analogWrite(speed_2, speed);
  analogWrite(speed_3, speed);
  analogWrite(speed_4, speed);
  delay(2000);
}
