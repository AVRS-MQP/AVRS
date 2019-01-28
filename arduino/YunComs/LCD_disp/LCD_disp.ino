#include <SparkFunSerialGraphicLCD.h>//inculde the Serial Graphic LCD library
#include <SoftwareSerial.h>

LCD LCD;
char message[]= "Select Vehicle to Simulate ";
int n = 0;
int pb[4] = {0,0,0,0}; //Push button top, right, bottom, left in order. 

void setup()
{
  delay(1200);///wait for the one second splash screen
//  LCD.restoreDefaultBaud();
  LCD.clearScreen();
  LCD.setHome();
  draw();
}

void loop()
{
//  delay(1000);
//  LCD.clearScreen();
  
//  if(n = 0) { LCD.printStr(message); }

//  if(n < 2) {LCD.printStr("Test"); }
//  LCD.drawLine(5,5,5,20,1);

//  n++;
}

void draw()
{
  
  LCD.printStr("Test");
  LCD.nextLine();
  LCD.printStr("Other");
  LCD.nextLine();
  LCD.printStr("Things");
  LCD.nextLine();
  LCD.printStr("Scren");
  LCD.nextLine();
  LCD.printStr(message);
  LCD.nextLine();
  LCD.printStr("Test");
//  LCD.printStr("Pleasework");
//  LCD.printStr("Please work"  "Is this working");
//  LCD.drawLine(5,5,5,20,1);
  
}

// https://github.com/sparkfun/GraphicLCD_Serial_Backpack/blob/master/Libraries/Arduino/src/SparkFunSerialGraphicLCD.cpp
