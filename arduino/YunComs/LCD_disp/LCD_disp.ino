#include <SparkFunSerialGraphicLCD.h>//inculde the Serial Graphic LCD library
#include <SoftwareSerial.h>

LCD LCD;
char message1[]= "Select Charger:";
char message2[]= "(L)-Tesla";
char message3[]= "(R)-J1772:";

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

  LCD.nextLine();
  LCD.printStr(message1);
  LCD.nextLine();
  LCD.printStr(message2);
  LCD.nextLine();
  LCD.printStr(message3);
//  LCD.printStr("(R)-J1772");
//  LCD.nextLine();

  
}

// https://github.com/sparkfun/GraphicLCD_Serial_Backpack/blob/master/Libraries/Arduino/src/SparkFunSerialGraphicLCD.cpp
