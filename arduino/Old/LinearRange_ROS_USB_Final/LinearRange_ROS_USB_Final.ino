/*
//Combination of ros serial publisher and linear range finder programs with topic error reporting
//Nikolas Gamarra
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher range_error("range_error", &str_msg);

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);
unsigned long range_timer;
char frameid[] = "/ir_ranger";

char hello[13] = "hello world!";

//sensor
#include <Wire.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
String errorMSG = "default";

void setup()
{
  nh.initNode();
  nh.advertise(range_error);

  nh.advertise(pub_range);


  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = -10;
  range_msg.max_range = 190;

  if (! vl.begin()) {
    //Serial.println("Failed to find sensor");
    errorMSG = "Failed to find sensor";
    int str_len = errorMSG.length() + 1;    // Length (with one extra character for the null terminator)
    char char_array[str_len];    // Prepare the character array (the buffer)
    errorMSG.toCharArray(char_array, str_len);    // Copy it over
    str_msg.data = char_array;
    range_error.publish( &str_msg );
    while (1);
  }
}
float range = 0;

void loop()
{

  float lux = vl.readLux(VL6180X_ALS_GAIN_5);

  // uint8_t
  range = vl.readRange();

  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    //Serial.print("Range: "); Serial.println(range);
    errorMSG = "No Error";
  }

  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    //Serial.println("System error");
    errorMSG = "No Error";
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    //Serial.println("ECE failure");
    errorMSG = "ECE failure";
    range = -1;
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    //Serial.println("No convergence");
    errorMSG = "No convergence";
    range = -2;
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    //Serial.println("Ignoring range");
    errorMSG = "Ignoring range";
    range = -3;
  }
  else if (status == VL6180X_ERROR_SNR) {
    //Serial.println("Signal/Noise error");
    errorMSG = "Signal/Noise error";
    range = -4;
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    //Serial.println("Raw reading underflow");
    errorMSG = "Raw reading underflow";
    range = -5;
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    //Serial.println("Raw reading overflow");
    errorMSG = "Raw reading overflow";
    range = -6;
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    //Serial.println("Range reading underflow");
    errorMSG = "Range reading underflow";
    range = -7;
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    // Serial.println("Range reading overflow");
    errorMSG = "Range reading overflow";
    range = -8;
  }
  int str_len = errorMSG.length() + 1;    // Length (with one extra character for the null terminator)
  char char_array[str_len];    // Prepare the character array (the buffer)
  errorMSG.toCharArray(char_array, str_len);    // Copy it over
  str_msg.data = char_array;
  range_error.publish( &str_msg );

  //some error correcting
 /* if (range = 255) {
    range = -10;
  }
*/
  float y = 0;
  /*if (range > 150) {
    y = map(range, 95, 187, 100, 190);
  } else {
    y = range;
  }*/
  y=range;

  range_msg.range = y;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);

  nh.spinOnce();
  delay(1000);
}
