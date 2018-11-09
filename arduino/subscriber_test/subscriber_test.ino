/*
  //AVRS-MQP Arduino Mega code for EOAT
  //Nikolas Gamarra & Ryan O'Brien
*/

//ROS includes
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <force_msgs/LoadCellForces32.h>

//sensor includes
#include <Wire.h>
#include "Adafruit_VL6180X.h"

ros::NodeHandle  nh;

   

//make msgs
std_msgs::String str_msg;
sensor_msgs::Range range_msg;
force_msgs::LoadCellForces32 Force_msg;

//make publishers
ros::Publisher pub_range_error("pub_range_error", &str_msg);
ros::Publisher pub_force( "force_data", &Force_msg);
ros::Publisher pub_range( "range_data", &range_msg);

char frameid[] = "/ir_ranger";
char frameid2[] = "/load_cell";

//range finder setup
Adafruit_VL6180X vl = Adafruit_VL6180X();
String errorMSG = "default";
float range = 0;

//definitions
#define loadCellA_pin 1//top
#define loadCellB_pin 2//clockwise
#define loadCellC_pin 3

void calibrationCb(const std_msgs::Int32& calibration_msg){//
  if(calibration_msg.data == 1){
    digitalWrite(LED_BUILTIN, HIGH);   // blink the led
  }else if (calibration_msg.data==0){
    digitalWrite(LED_BUILTIN, LOW);   // turn led off

  }
}//end calibrationCb

//make subscriber
ros::Subscriber<std_msgs::Int32> sub_calibration("force_calibration", &calibrationCb);

void setup()
{

    pinMode(LED_BUILTIN, OUTPUT);

  nh.initNode();//initialize the ros node

//setup subscriber
    nh.subscribe(sub_calibration);


  //advertise publishers
  nh.advertise(pub_range_error);
  nh.advertise(pub_range);
  nh.advertise(pub_force);

  //nh.getHardware()->setBaud(2000000);  //comment out to use dedault baud

  //setup force msg
  Force_msg.header.frame_id = frameid2;

  //setup range msg
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
    pub_range_error.publish( &str_msg );
    while (1);
  }
}//end setup

void loop()
{

  float lux = vl.readLux(VL6180X_ALS_GAIN_5);//get light value

  range = vl.readRange();//get range value
  checkRangeError();//overwrites range if error detected

  //set load cell msg valuse
  Force_msg.header.stamp = nh.now();
  Force_msg.cellA = 5;
  Force_msg.cellB = 0;
  Force_msg.cellC = 10;

  //set range msg
  range_msg.header.stamp = nh.now();
  range_msg.range = range;


  //setup error msg
  int str_len = errorMSG.length() + 1;    // Length (with one extra character for the null terminator)
  char char_array[str_len];    // Prepare the character array (the buffer)
  errorMSG.toCharArray(char_array, str_len);    // Copy it over
  str_msg.data = char_array;
  
  //publish everything
  pub_range.publish(&range_msg);
  pub_force.publish(&Force_msg);
  pub_range_error.publish( &str_msg );

  nh.spinOnce();
  delay(400);//1000
}//end loop



void checkRangeError() { //checks adafruit lib for errors and overites range if present
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

}//end checkRangeError
