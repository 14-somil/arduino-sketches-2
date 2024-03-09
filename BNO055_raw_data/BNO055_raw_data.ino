#include <Wire.h>

#define BNO055_ADDR 0x28 
#define PAGE_ID_REG 0x07
#define DATA_REG 0x20

int16_t qua_x_bytes, qua_y_bytes, qua_z_bytes, qua_w_bytes, lia_x_bytes, lia_y_bytes, lia_z_bytes;
float qua_x, qua_y, qua_z, qua_w, lia_x, lia_y, lia_z;
const float BYTE_2_QUA = 1/pow(2,14);
const float BYTE_2_LIA = 0.01;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(PAGE_ID_REG); 
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(DATA_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(BNO055_ADDR, 14, true);
  qua_w_bytes = (Wire.read()<<8)|Wire.read();
  qua_x_bytes = (Wire.read()<<8)|Wire.read();
  qua_y_bytes = (Wire.read()<<8)|Wire.read();
  qua_z_bytes = (Wire.read()<<8)|Wire.read();
  lia_x_bytes = (Wire.read()<<8)|Wire.read();
  lia_y_bytes = (Wire.read()<<8)|Wire.read();
  lia_z_bytes = (Wire.read()<<8)|Wire.read();

  qua_w = (float)qua_w_bytes * BYTE_2_QUA;
  qua_x = (float)qua_x_bytes * BYTE_2_QUA;
  qua_y = (float)qua_y_bytes * BYTE_2_QUA;
  qua_z = (float)qua_z_bytes * BYTE_2_QUA;

  lia_x = (float)lia_x_bytes * BYTE_2_LIA;
  lia_y = (float)lia_y_bytes * BYTE_2_LIA;
  lia_z = (float)lia_z_bytes * BYTE_2_LIA;

  Serial.print("w=");
  Serial.print(qua_w);
  Serial.print(" x=");
  Serial.print(qua_x);
  Serial.print(" y=");
  Serial.print(qua_y);
  Serial.print(" z=");
  Serial.print(qua_z);
  Serial.print(" ax=");
  Serial.print(lia_x);
  Serial.print(" ay=");
  Serial.print(lia_y);
  Serial.print(" az=");
  Serial.println(lia_z);
  delay(1000);
}
