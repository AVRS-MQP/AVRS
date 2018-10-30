#include <Console.h>

const int ledPin = 13; // the pin that the LED is attached to
int incomingByte;      // a variable to read incoming serial data into
char car1[6] = "Tesla";
char car2[5] = "Volt";
char car3[5] = "Leaf";
char carSel[6];

void setup() {
  // initialize serial communication:
  Bridge.begin();
  Console.begin(); 

  while (!Console){
    ; // wait for Console port to connect.
  }
  Console.println("Now connected to the vehicle");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // see if there's incoming serial data
  if (Console.available() > 0) { // read the oldest byte in the serial buffer:

    //For testing if terminal is outputting correctly
    incomingByte = Console.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H' || incomingByte == 'h') {
      digitalWrite(ledPin, HIGH);
    } 
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == 'L' || incomingByte == 'l') {
      digitalWrite(ledPin, LOW);
    }

    //Opens gas flap
    if(incomingByte == 'O' || incomingByte == 'o') {
      Console.println("Opening gas flap");
      //TODO: vary two digital pin voltage to open the flap
    }

    //Asks for vehicle information for one of the three models 
    //May expand later, right now only needs proof of concept to get info on demand
    if(incomingByte == '1' || incomingByte == '2' || incomingByte == '3') {
      Console.println("Recieving vehicle information");
      
      delay(200);
      
    }

  }
  delay(100);
}


char* vehicle_type(int sel) {
  
}

  
