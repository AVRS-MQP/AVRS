

//DIRECTIONS:
//DOWN: -A, +B, +C
//UP: +A, -B, -C
//LEFT: -A, +B, -C
//RIGHT: -A, -B, +C

#include "HX711.h"

#define DOUT_A  12
#define CLK_A  11
#define DOUT_B  10
#define CLK_B  9
#define DOUT_C  8
#define CLK_C  7

HX711 cellA(DOUT_A, CLK_A);
HX711 cellB(DOUT_B, CLK_B);
HX711 cellC(DOUT_C, CLK_C);

float calibration_factorA = -10340.0; 
float calibration_factorB = -10300.0;
float calibration_factorC = -10330.0;

//Gathered from physical calibration, use these in other programs
long zero_factorA = 27369;
long zero_factorB = 1250;
long zero_factorC = 6000;

void setup() {
  
  Serial.begin(9600);
  Serial.println("Demo of zeroing out a scale from a known value");

  cellA.set_scale(calibration_factorA);
  cellA.set_offset(zero_factorA); //can cellA.tare() if needed
  cellA.tare();
  
  cellB.set_scale(calibration_factorB); 
  cellB.set_offset(zero_factorB);
  cellB.tare();
  
  cellC.set_scale(calibration_factorC); 
  cellC.set_offset(zero_factorC);
  cellC.tare();

  Serial.println("Readings:");
}

void loop() {
  
  Serial.print("Cell A: ");
  Serial.print(cellA.get_units(), 1); //scale.get_units() returns a float
  Serial.print(" lbs  "); 

  Serial.print("Cell B: ");
  Serial.print(cellB.get_units(), 1); //scale.get_units() returns a float
  Serial.print(" lbs  "); 

  Serial.print("Cell C: ");
  Serial.print(cellC.get_units(), 1); //scale.get_units() returns a float
  Serial.print(" lbs"); 
  Serial.println();


}
