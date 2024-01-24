
#define dir_1 52
#define dir_2 7
// #define dir_3 12
// #define dir_4 13
#define speed_1 9
#define speed_2 11
// #define speed_3 5
// #define speed_4 3

int speed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  // pinMode(dir_3, OUTPUT);
  // pinMode(dir_4, OUTPUT);
  pinMode(speed_1, OUTPUT);
  pinMode(speed_2, OUTPUT);

  digitalWrite(dir_1, HIGH);
        digitalWrite(dir_2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
      Serial.println(speed);
            analogWrite(speed_1, speed);
      speed += 1;
      delay(100);
}
