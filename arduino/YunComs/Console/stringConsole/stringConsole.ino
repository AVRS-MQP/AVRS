#include <SparkFunSerialGraphicLCD.h>//inculde the Serial Graphic LCD library
#include <SoftwareSerial.h>
#include <Console.h>

LCD LCD;
char message1[]= "Select Charger:";
char message2[]= "(L)-Tesla";
char message3[]= "(R)-J1772:";
int n = 0;
int pb[4] = {0,0,0,0}; //Push button top, right, bottom, left in order.
const int ledPin = 13; // the pin that the LED is attached to
int incomingByte;      // a variable to read incoming serial data into

//Models of vehicles
char car1[6] = "Tesla";
char car2[6] = "Volt";
char car3[6] = "Leaf";

//Types of charging standards
char charge1[8] = "tesla";
char charge2[8] = "J1772";
char charge3[8] = "CHAdeMO";

//Charging levels
int lvl1 = 1;
int lvl2 = 2;
int lvl3 = 3;

bool flap_auto_open;
bool flap_unlocked;

char carSel[6]; //Holds string for which vehicle is selected
char chargeSel[8];  //Holds string for which charger matches the vehicle
int battPcnt; //Will hold random percentage of 'battery' charge
int chgLvl; //Holds maximum charging level


void setup() {
  // initialize serial communication:
  Bridge.begin();
  Console.begin(); 
  LCD.clearScreen();
  LCD.setHome();

  while (!Console){
    ; // wait for Console port to connect.
  }
  Console.println("Now connected to the vehicle");
  
  draw();
  //TODO: button logic to select charger
  
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

    //Toggle gas flap locked/unlocked
    if(incomingByte == 'f' || incomingByte == 'F') {
      Console.println("Opening gas flap");
      //TODO: vary two digital pin voltage to open the flap
      flap_unlocked = !flap_unlocked;
      Console.println(flap_unlocked);
    }

    //Asks for vehicle information for one of the three models 
    //May expand later, right now only needs proof of concept to get info on demand
    if(incomingByte == '1' || incomingByte == '2' || incomingByte == '3') { //Should later be set to push buttons or other physical input
      Console.println("Recieving vehicle information");
      vehicle_type(incomingByte);
      delay(100);
      Console.println(carSel);
      Console.println(chargeSel);
      Console.println(battPcnt);
      Console.println(chgLvl);
      Console.println(flap_auto_open);

    }

  }
  delay(10);
}

//User input selects between three models of vehicles based on which female charger we have slotted 
void vehicle_type(char sel) {

  battPcnt = random(1,90); //randomly sets the simulated vehicle's battery charge percentage
  
  if(sel == '1') { //Tesla
    strcpy(carSel, car1);
    strcpy(chargeSel, charge1);
    chgLvl = lvl3;
    flap_auto_open = true;
    LCD.printStr("tesla selected");
  }
  else if (sel == '2') { //Volt
    strcpy(carSel, car2);
    strcpy(chargeSel, charge2);
    chgLvl = lvl2;
    flap_auto_open = false;
    LCD.printStr("volt selected");
  }
  else { //Leaf
    strcpy(carSel, car3);
    strcpy(chargeSel, charge3);
    chgLvl = lvl2; //May also support level 3, needs confirmation
    flap_auto_open = false;
    LCD.printStr("volt selected");

  }

}
 
void draw()
{
 
  LCD.printStr(message1);
  LCD.nextLine();
  LCD.printStr(message2);
  LCD.nextLine();
//  LCD.printStr(message3);
//  LCD.printStr("(R)-J1772");
//  LCD.nextLine();

  
}
