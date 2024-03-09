#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

#define BNO055_ADDR 0x28 
#define PAGE_ID_REG 0x07
#define DATA_REG 0x20

ros::NodeHandle nh;

sensor_msgs::Imu msg;
ros::Publisher pub("/IMU", &msg);

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
  
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(pub);

}

void loop() {
  // // put your main code here, to run repeatedly:
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

  msg.orientation.w = qua_w;
  msg.orientation.x = qua_x;
  msg.orientation.y = qua_y;
  msg.orientation.z = qua_z;

  msg.linear_acceleration.x = lia_x;
  msg.linear_acceleration.y = lia_y;
  msg.linear_acceleration.z = lia_z;

  pub.publish(&msg);
  delay(10);
  nh.spinOnce();
}
