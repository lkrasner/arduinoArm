#include "math.h"


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
double baseLiftPos=87;
double elbowPos=90;
double wristPos=89;
double wristRotatePos=90;
double gripPos=90;
double distanceExtension=27.5;
double distanceExtensionSpeed=0;
double zSpeed=0;
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
double angleA =0;
double angleA2 = 0;
double angle3 = 0;
double zDistance = 22; 
int wristServoOffset = 70;
boolean moveFreely=false;



void setup()  //sets up servos and starts serial communication proccess
{
  baseRotate.attach(41, 500, 2700);  //"attaches" servos to correct output pins
  baseLift.attach(39);
  elbow.attach(43);
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

  if (Usb.Init() == -1) { //does some usb set up thing
    //Serial.print(F("\r\nOSC did not start"));
    while(1); //halt
  }
  //no idea what it is, doesn't work without it.  


  do{
    Usb.Task();
  }
  while(PS3.PS3Connected==false);
  Serial.begin(9600);
  //Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void loop() //calls all the other functions in the proper order
{

  Usb.Task(); //no idea what it is, doesn't work without it.  

  // if(PS3.PS3Connected)
  //{
  mapAnalog();
  setSpeeds();
  getPowerAndGrip();
  setPositions();
  writeServos();
  Serial.print(angleA);
  Serial.print(",");
  Serial.print(angleA2);
  Serial.print(",");
  Serial.print(distanceExtension);
  Serial.print(",");
  Serial.print(zDistance);
  Serial.print(",");
  Serial.print("z");
  delay(waitTime);  //waits for a given time to keep the speeds in check
  // }

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

void setSpeeds() //sets the speeds and deals with dead zones
{
  if((leftX>deadZoneHigh)||(deadZoneLow>leftX)) 
  {
    baseRotateSpeed = calculateSpeed(leftX, baseRotatePos)/2;
  }
  else
  {
    baseRotateSpeed = 0;
  }

  if((leftY>deadZoneHigh)||(deadZoneLow>leftY)) 
  {
    distanceExtensionSpeed = ((calculateSpeed(leftY, baseLiftPos))*.05);

    if(((distanceExtension+distanceExtensionSpeed)<56.24)&&((distanceExtension+distanceExtensionSpeed)>0))
    {
      distanceExtension=distanceExtension+distanceExtensionSpeed;
      angleA =getAngle(toDegree(getA(distanceExtension,zDistance)));

      angleA2 = getAngle(toDegree(calcAngle2(distanceExtension,getA(distanceExtension,zDistance))));
      angle3=calcAngle3(angleA,angleA2);  
    }

    if(angleA<30)
    {
      angleA=30;
      angleA2 = getAngle(toDegree(calcAngle2(distanceExtension,.5235987756)));
      angle3=calcAngle3(angleA,angleA2);  

    }

    if(((angleA)<180)&&((angleA)>0)&&(checkAngles()==true))
    {
      baseLiftPos = angleA;
    }

    if(((angleA2)<180)&&(angleA2>0)&&(checkAngles()==true))
    {
      elbowPos = angleA2;
    }
    if(((angleA2)<180)&&(angleA2>0)&&(checkAngles()==true))
    {
      wristPos = angle3;
    }

  }
  else  
  {
    distanceExtensionSpeed = 0;
  }

  if((rightX>deadZoneHigh)||(deadZoneLow>rightX)){
    wristRotateSpeed = calculateSpeed(rightX, wristRotatePos);
  }
  else 
  {
    wristRotateSpeed = 0;
  }
  if((rightY>deadZoneHigh)||(deadZoneLow>rightY)) {

    zSpeed = ((calculateSpeed(rightY, baseLiftPos))*.05);
    if(zCheck(zDistance+zSpeed)==true)
    {
      zDistance=zDistance+zSpeed;
      angleA =getAngle(toDegree(getA(distanceExtension,zDistance)));

      angleA2 = getAngle(toDegree(calcAngle2(distanceExtension,getA(distanceExtension,zDistance))));
      angle3 = calcAngle3(angleA,angleA2);  
    }

    if(angleA<30)
    {
      angleA=30;
      angleA2 = getAngle(toDegree(calcAngle2(distanceExtension,.5235987756)));
      angle3 = calcAngle3(angleA,angleA2);  

    }
    if(((angleA)<180)&&((angleA)>0)&&(checkAngles()==true))
    {
      baseLiftPos = angleA;
    }
   
    if(((angleA2)<180)&&(angleA2>0)&&(checkAngles()==true))
    {
      elbowPos = angleA2;
    }
    if(((angleA2)<180)&&(angleA2>0)&&(checkAngles()==true))
    {
      wristPos = angle3;
    }

  }
  else  
  {
    distanceExtensionSpeed = 0;
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
  baseRotate.write(baseRotatePos-6);
  baseLift.write(baseLiftPos+25);
  elbow.write(elbowPos+12);
  wrist.write(wristPos-64);
  wristRotate.write(wristRotatePos-17);
  grip.write(gripPos);
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


double getA(double d, double z)
{
  double root = (((-16*(pow(d,4))-32*(sq(d))*(sq(z))-736*(sq(d))*z+60296*(sq(d))-16*(pow(z,4))-736*(pow(z,3))+51832*(sq(z))+1.38681*(pow(10,6))*z+4.18973*(pow(10,6)))+284*z+3266)/(4*(sq(d))+284*d+4*(sq(z))+92*z+2545));
  double taninverse = ((sqrt(-16*(pow(d,4))-32*(sq(d))*(sq(z))-736*(sq(d))*z+60296*(sq(d))-16*(pow(z,4))-736*(pow(z,3))+51832*(sq(z))+1.38681*(pow(10,6))*z+4.18973*(pow(10,6)))+284*z+3266)/(4*(sq(d))+284*d+4*(sq(z))+92*z+2545));

  double a = 2*(atan((sqrt(-16*(pow(d,4))-32*(sq(d))*(sq(z))-736*(sq(d))*z+60296*(sq(d))-16*(pow(z,4))-736*(pow(z,3))+51832*(sq(z))+1.38681*(pow(10,6))*z+4.18973*(pow(10,6)))+284*z+3266)/(4*(sq(d))+284*d+4*(sq(z))+92*z+2545))+3.141593);
  if((taninverse>=1)||(root<=0))
  {
    return (toRadians(baseLiftPos+(((-calculateSpeed(leftY, baseLiftPos)))*.15)));

  }

  return a;
}
double toDegree(double rad)
{
  return ((rad/3.1415923)*180);
}
double getAngle(double angle)
{
  double returnAngle =angle;
  if(angle>180)
  {
    while(angle>180)
    {
      angle = angle-360;
    }
    returnAngle =angle;
  }
  else if(angle<0)
  {
    while(angle<0)
    {
      angle = angle+360;
    }
    returnAngle =angle;
  }
  return returnAngle;

}

double calcAngle2(double d,double angleARad)
{
  if(angleARad<0)
  {
    angleARad=angleARad+(3.141592);            
  }
  double angle2 = ((-acos((d-cos(angleARad)*35.5)/27.5))+3.14159-angleARad);
  if((((d-cos(angleARad)*35.5)/27.5)>=1))
  {
    return 1.570796;
  }
  else
  {
    return angle2;
  }
  
}
double calcAngle3(double angle1,double angle2)
{
  return(270-angle1-angle2);
}
double toRadians(double degree)
{
  return ((degree/180)*3.1415926);
}
boolean zCheck(double zDistance)
{
  double maxZ= sin(toRadians(angleA))*(27.5+35.5);

  if((zDistance>maxZ)||(zDistance<-10))
  {
    return false;
  }
  else
  {
    return true;
  }
}

/**
 *  checks for bugs in the equation of motion to move in a plane
 */
boolean checkAngles()
{
  if(((angleA+angleA2)>=178)||(angleA>88))
  {
    
    return false;
  }
  else
  {  
    return true;
  }

}


