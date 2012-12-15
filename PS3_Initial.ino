/**
Code By: Luke Krasner and David Groden
Email: luke.krasner@gmail.com
Website: https://lukerasner.com
Github https://github.com/lkrasner
This is experimental code at this point and may cause unexpected results.
It is recomended that you exaamine and test the code before assuming it will work.
I take no responsibilty for any damage this may cause to hardware due to malfunctions.
**/

#include <Boards.h>
#include <Firmata.h>
#include <Servo.h> //Has stuff neccesary for servo use
#include <PS3BT.h> //Has the PS3 Controller stuff

USB Usb; //Sets up the usb host stuff
BTD Btd(&Usb);
PS3BT PS3(&Btd); // Creates the PS3 control instance

Servo baseRotate; // Creates the servo objects
Servo baseLift;
Servo elbow;
Servo wrist;
Servo wristRotate;
Servo grip;

const int deadZoneHigh = 145;  //deadzones for the Controller
const int deadZoneLow = 105;
boolean powerOn = false;  //Is the power on?
int powerPin=13;  //Pin To be used for toggling the power to the servos
double baseRotatePos=90; //Positions the servos should move to.
double baseLiftPos=90;
double elbowPos=90;
double wristPos=90;
double wristRotatePos=90;
double gripPos=90;
int leftX; //Controller stick positions mapped from -128 to 128
int leftY;
int rightX;
int rightY;
int leftTrigger; //controller triggers (L2 and R2) mapped 0-255
int rightTrigger;
double baseRotateSpeed;  //Servo speeds.  0 to 6 degrees at a time
double baseLiftSpeed;
double elbowSpeed;
double wristSpeed;
double wristRotateSpeed;
const int servoDelay = 6;  //How long to delay after writing to the servos.  This also is a factor in the speed.


void setup()  //Sets up servos, PS3 service, and starts serial communication proccess
{
  baseRotate.attach(41, 500, 2700);  //Attaches Servos to pins and sets the range of pulse times properly for each servo.
  baseLift.attach(39, 900, 2100);
  elbow.attach(43, 900, 2100);
  wrist.attach(45, 600, 2500);
  wristRotate.attach(47, 600, 2500);
  grip.attach(49, 500, 2400);

  pinMode(powerPin, OUTPUT); //Sets the power control pin as an output
  digitalWrite(powerPin, LOW);  //Turns the power off to begin with  
  Serial.begin(9600);  //Starts serial service for debugging purposes

  if (Usb.Init() == -1) { //I think this is important
    Serial.print(F("\r\nOSC did not start"));
    while(1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

boolean psClicked() //Is the PS button clicked
{
  if(PS3.getButtonClick(PS)) return true;
  else return false;
}

boolean l1Clicked() //L1
{
  if(PS3.getButtonClick(L1)) return true;
  else return false;
}

boolean r1Clicked() //R1
{
  if(PS3.getButtonClick(R1)) return true;
  else return false;
}

int getAnalogLeftX() //Theses are all for analog values of the sticks and triggers.  also handles dead zones
{
  int localAnalog = PS3.getAnalogHat(LeftHatX);
  if((localAnalog<deadZoneLow)||(deadZoneHigh<localAnalog)) return (localAnalog);
  else return 128;
}

int getAnalogLeftY()
{
  int localAnalog = PS3.getAnalogHat(LeftHatY);
  if((localAnalog<deadZoneLow)||(deadZoneHigh<localAnalog)) return (localAnalog);
  else return 128;
}

int getAnalogRightX()
{
  int localAnalog = PS3.getAnalogHat(RightHatX);
  if((localAnalog<deadZoneLow)||(deadZoneHigh<localAnalog)) return (localAnalog);
  else return 128;
}

int getAnalogRightY()
{
  int localAnalog = PS3.getAnalogHat(RightHatY);
  if((localAnalog<deadZoneLow)||(deadZoneHigh<localAnalog)) return (localAnalog);
  else return 128;
}

int getAnalogButtonLeft()
{
  int localAnalog = PS3.getAnalogButton(L2_ANALOG);
  return (localAnalog);
}

int getAnalogButtonRight()
{
  int localAnalog = PS3.getAnalogButton(R2_ANALOG);
  return (localAnalog);
}

void mapAnalog() //called to map the sticks and buttons with negatives
{
  leftX = map(getAnalogLeftX(),0,255,-128,128);
  leftY = map(getAnalogLeftY(),0,255,-128,128);
  rightX = map(getAnalogRightX(),0,255,-128,128);
  rightY = map(getAnalogRightY(),0,255,-128,128);
  leftTrigger = map(getAnalogButtonLeft(),0,255,0,128);
  rightTrigger = map(getAnalogButtonRight(),0,255,0,128);
}

double calculateSpeed(int currentAnalog, int pos) //function to calculate the speed with a power function for better control and speed
{
  double a; //doubles allow decimal values for more acuracy when you only have 0-6 degrees.
  if(currentAnalog > 0 && pos < 180)
    a = (pow(0.0156*currentAnalog,2.59095));
  else if(currentAnalog < 0  && pos > 0)
    a = 0 - (pow(abs(0.0156*currentAnalog),2.59095));
  return a;
}
void setSpeeds() //sets the speeds, using the above function
{
  if(leftX != 0) baseRotateSpeed = calculateSpeed(leftX, baseRotatePos);
  else baseRotateSpeed = 0;  

  if(leftY != 0) baseLiftSpeed = calculateSpeed(leftY, baseLiftPos);
  else baseLiftSpeed = 0;

  if(rightX != 0) wristRotateSpeed = calculateSpeed(rightX, wristRotatePos);
  else wristRotateSpeed = 0;

  if(rightY != 0) elbowSpeed = calculateSpeed(rightY, elbowPos);
  else elbowSpeed = 0;

  if(leftTrigger != 0) wristSpeed = calculateSpeed(leftTrigger, wristSpeed);
  else wristSpeed = 0;

  if(rightTrigger != 0) wristSpeed = (0 - calculateSpeed(rightTrigger, wristSpeed));
  else wristSpeed = 0;
  

}

void getPowerAndGrip()  //deals with the buttons for the power and gripper
{
  boolean togglePower = psClicked();
  if(togglePower == true) digitalWrite(powerPin, !digitalRead(powerPin));  //Little trick to allow toggling the power easily.  Will ALWAYS switch the state of the pin, not just on or off.
  boolean openGripper = l1Clicked();
  boolean closeGripper = r1Clicked();
  if(openGripper == true) gripPos = 80; //the positions the gripper should be in for open/ closed
  else if(closeGripper == true) gripPos = 180;
}

void writeServos()  //Sets the position and writes the servos
{
  baseRotatePos+= baseRotateSpeed;
  baseRotate.write(baseRotatePos);
  delay(servoDelay);
  Serial.println(baseRotatePos);

  baseLiftPos+= baseLiftSpeed;
  baseLift.write(baseLiftPos);
  delay(servoDelay);

  elbowPos+= elbowSpeed;
  elbow.write(elbowPos);
  delay(servoDelay);

  wristPos+= wristSpeed;
  wrist.write(wristPos);
  delay(servoDelay);

  wristRotatePos+= wristRotateSpeed;
  wristRotate.write(wristRotatePos);
  delay(servoDelay);

  grip.write(gripPos);
  delay(servoDelay);
}

void sendData() //left in for debugging purposes, just throw some stuf in to be printed out
{
  Serial.println();
}
void loop()  //only calls other stuff
{

  Usb.Task(); //Needed to handle USB stuff each time around

  if(PS3.PS3Connected)  //only go on if the controller is connected
  {
    mapAnalog();
    setSpeeds();
    getPowerAndGrip();
    writeServos();
    //sendData();
  }

}










