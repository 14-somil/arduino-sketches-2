/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::String& msg){
  //Serial.println(msg.data);
  
}

ros::Subscriber<std_msgs::String> sub("chatter", &messageCb );

void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  //Serial.begin(9600);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

