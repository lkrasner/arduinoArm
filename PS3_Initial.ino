#include <Boards.h>
#include <Firmata.h>
#include <Servo.h> //has stuff neccesary for servo use
#include <PS3BT.h> //has the PS3 Controller stuff

USB Usb; //sets up the usb host stuff
BTD Btd(&Usb);
PS3BT PS3(&Btd); // Creates the PS3 control instance

Servo baseRotate; //the servo objects
Servo baseLift;
Servo elbow;
Servo wrist;
Servo wristRotate;
Servo grip;

const int deadZoneHigh = 30; //deadzones for the sticks
const int deadZoneLow = -30;
int powerPin=3; //pin to be used for the power togle
double baseRotatePos=90; //posistions for the servos
double baseLiftPos=90;
double elbowPos=90;
double wristPos=90;
double wristRotatePos=90;
double gripPos=90;
int leftX;  //positions of sticks and triggers. -127 to 127
int leftY;
int rightX;
int rightY;
int leftTrigger;
int rightTrigger;
double baseRotateSpeed;
double baseLiftSpeed;
double elbowSpeed;
double wristSpeed;
double wristRotateSpeed;
const int waitTime = 50;


void setup()  //sets up servos and starts serial communication proccess
{
  baseRotate.attach(41, 500, 2700);  //"attaches" servos to correct output pins
  baseLift.attach(39, 900, 2100);
  elbow.attach(43, 900, 2100);
  wrist.attach(45, 600, 2500);
  wristRotate.attach(47, 600, 2500);
  grip.attach(49, 500, 2400);
  baseRotate.write(90); //sets the servos to 90 to begin with
  baseLift.write(90);
  elbow.write(90);
  wrist.write(90);
  wristRotate.write(90);
  grip.write(90);

  pinMode(powerPin, OUTPUT); //sets the power control pin as an output
  digitalWrite(powerPin, LOW); //turns the power off
  Serial.begin(9600);  //starts serial service for debugging purposes

  if (Usb.Init() == -1) { //does some usb set up thing
    Serial.print(F("\r\nOSC did not start"));
    while(1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void loop() //calls all the other functions in the proper order
{

  Usb.Task(); //no idea what it is, doesn't work without it.  

  if(PS3.PS3Connected)
  {
    mapAnalog();
    setSpeeds();
    getPowerAndGrip();
//    getPositions();
    writeServos();
    delay(waitTime);  //waits for a given time to keep the speeds in check
  }

}

//all these booleans are just whether something has been clicked or pressed
boolean l1Clicked()
{
  if(PS3.getButtonClick(L1))
  {
    return true;
  }
  else
  {
    return false;
  }
}

boolean r1Clicked()
{
  if(PS3.getButtonClick(R1))
  {
    return true;
  }
  else
  {
    return false;
  }
}

boolean psClicked()
{
  if(PS3.getButtonClick(PS))
  {
    return true;
  }
  else
  {
    return false;
  }
}

//Raw analog values for the sticks and triggers.
int analogLeftX()
{
  int localAnalog = PS3.getAnalogHat(LeftHatX);

  return (localAnalog);

}

int analogLeftY()
{
  int localAnalog = PS3.getAnalogHat(LeftHatY);

  return (localAnalog);

}

int analogRightX()
{
  int localAnalog = PS3.getAnalogHat(RightHatX);

  return (localAnalog);

}

int analogRightY()
{
  int localAnalog = PS3.getAnalogHat(RightHatY);

  return (localAnalog);


}

int analogButtonLeft()
{
  int localAnalog = PS3.getAnalogButton(L2_ANALOG);
  return (-localAnalog);
}

int analogButtonRight()
{
  int localAnalog = PS3.getAnalogButton(R2_ANALOG);
  return (localAnalog);
}

void mapAnalog() //maps the raw values to a range of -127 to 127 allowing for easy direction control
{
  leftX = map(analogLeftX(),0,255,-127,127);
  leftY = map(analogLeftY(),0,255,-127,127);
  rightX = map(analogRightX(),0,255,-127,127);
  rightY = map(analogRightY(),0,255,-127,127);
  leftTrigger = map(analogButtonLeft(),-255,0,-127,0);
  rightTrigger = map(analogButtonRight(),0,255,0,127);
}

double calculateSpeed(int currentAnalog, int pos) //calculates the speed using an exponential curve
{
  double a;
  if((currentAnalog >= 0) && (pos < 180))
  {
    a = ((pow((currentAnalog/60.0),2.38)));
  }
  else if((currentAnalog < 0)  && (pos > 0))
  {
    a = 0 - ((pow((abs(currentAnalog)/60.0),2.38)));
  }
  return a;
}

void setSpeeds() //sets the speeds and deals with dead zones
{
  if((leftX>deadZoneHigh)||(deadZoneLow>leftX)) 
  {
    baseRotateSpeed = calculateSpeed(leftX, baseRotatePos);
  }
  else
  {
    baseRotateSpeed = 0;
  }

  if((leftY>deadZoneHigh)||(deadZoneLow>leftY)) 
  {
    baseLiftSpeed = calculateSpeed(leftY, baseLiftPos);
  }
  else  
  {
    baseLiftSpeed = 0;
  }

  if((rightX>deadZoneHigh)||(deadZoneLow>rightX)){
    wristRotateSpeed = calculateSpeed(rightX, wristRotatePos);
  }
  else 
  {
    wristRotateSpeed = 0;
  }
  if((rightY>deadZoneHigh)||(deadZoneLow>rightY)) {
    elbowSpeed = calculateSpeed(rightY, elbowPos);
  }
  else 
  {
    elbowSpeed = 0;
  }

  if((leftTrigger>deadZoneHigh)||(deadZoneLow>leftTrigger)) 
  {
    leftTrigger =-leftTrigger;
    wristSpeed = calculateSpeed(leftTrigger, wristSpeed);
    wristSpeed = (wristSpeed);
  }

  if((rightTrigger>deadZoneHigh)||(deadZoneLow>rightTrigger)) 
  {
    wristSpeed = 0-calculateSpeed(rightTrigger, wristSpeed);
  }
  if((rightTrigger<deadZoneHigh)&&(deadZoneLow<rightTrigger)&&((leftTrigger<deadZoneHigh)&&(deadZoneLow<leftTrigger)))
  {
    wristSpeed = 0;
  }

}

void getPowerAndGrip()  //gets stuff for power and gripper
{
  boolean togglePower = psClicked();
  if(togglePower == true) 
  {
    digitalWrite(powerPin, !digitalRead(powerPin));
  }

  boolean openGripper = l1Clicked();
  boolean closeGripper = r1Clicked();
  if(openGripper == true) 
  {
    gripPos = 80;
  }
  else if(closeGripper == true)
  { 
    gripPos = 180;
  }
}

void setPositions()  //updates positions as long as the updated value would not put them outside of thier range (0-180)
{
  if(((baseRotatePos+ baseRotateSpeed)<180)&&((baseRotatePos+ baseRotateSpeed)>0))
  {
    baseRotatePos+= baseRotateSpeed;
  }
  if(((baseLiftPos+ baseLiftSpeed)<180)&&((baseLiftPos+ baseLiftSpeed)>0))
  {
    baseLiftPos+= baseLiftSpeed;
  }
  if(((elbowPos+ elbowSpeed)<180)&&((elbowPos+ elbowSpeed)>0))
  {
    elbowPos+= elbowSpeed;
  }
  if(((wristPos+ wristSpeed)<180)&&((wristPos+ wristSpeed)>0))
  {
    wristPos+= wristSpeed;
  }
  if(((wristRotatePos+ wristRotateSpeed)<180)&&((wristRotatePos+ wristRotateSpeed)>0))
  {
    wristRotatePos+= wristRotateSpeed;
  }
}

void writeServos()  //writes the servos to the new positions
{
  baseRotate.write(baseRotatePos);
  baseLift.write(baseLiftPos);
  elbow.write(elbowPos);
  wrist.write(wristPos);
  wristRotate.write(wristRotatePos);
  grip.write(gripPos);
}
