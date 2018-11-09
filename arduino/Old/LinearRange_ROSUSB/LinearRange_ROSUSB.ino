//Combination of ros serial publisher and linear range finder programs
//Nikolas Gamarra
//include linear range finder stuff
#include <Wire.h>
#include "Adafruit_VL6180X.h"
//include ROS Stuff
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>


//setup range finder
Adafruit_VL6180X vl = Adafruit_VL6180X();

//setup ros node
ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
std_msgs::String str_msg;

ros::Publisher pub_range( "range_data", &range_msg);
ros::Publisher pub_range_error("range_error", &str_msg);


char frameid[] = "/ir_ranger";

unsigned long range_timer;



void setup() {
//Range finder
/*
  Serial.begin(115200);
  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
 
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
*/
  //ROS
  nh.initNode();
  nh.advertise(pub_range);
    nh.advertise(pub_range);

  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = -5;
  range_msg.max_range = 190;
}


void loop() {
float range;
  uint8_t status = vl.readRangeStatus();

  //  
  if (status == VL6180X_ERROR_NONE){
  range=vl.readRange();
  }else{
    range=-1;
  }

  if ( (millis()-range_timer) > 50){  // publish the range value every 50 milliseconds since it takes that long for the sensor to stabilize
    range_msg.range = range;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer =  millis() + 50;
  }
  nh.spinOnce();

  
  /*
  float lux = vl.readLux(VL6180X_ALS_GAIN_5);

  Serial.print("Lux: "); Serial.println(lux);
  
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);
  }

  // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
  delay(50);

  */
}
