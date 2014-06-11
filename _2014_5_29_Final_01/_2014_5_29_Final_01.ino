/* Prototype of my Graduation project.
 * Work prinzip: GCode parser read and understand the command from serial port. If this is the
 * movement command, inverse kenamatic translate the target position into new length of each 
 * wire that connected to different motors. Bresenham algorithmus coordinate the motor and move
 * them at last.
 * All input units converts to mm innerly in the whole arduino system. input unit can be mm or inch
 *
 * 6 Axis motor control with interpreter.
 * https://github.com/MarginallyClever/GcodeCNCDemo/blob/master/GcodeCNCDemo4axisV2/GcodeCNCDemo4AxisV2.ino
 *
 * Written: Gfast
 * last edite: 2014-6-2
 *
 */

#include "guang.h"
#include "stepperG.h"
#include "pins.h"

#include <inttypes.h>
#include <avr/io.h>
#include "stepperG.h"

//this part was included in Gfaststepper library. But this library didn't compiled correctly.
//So I copy the contente directely here.
//#include "Gfaststepper.h" 
#include <stdlib.h>
class Gfaststepper {
public:
	//Constructor
	Gfaststepper(uint8_t puls = 29, uint8_t direction = 30);
	
	//move one step belong the "direction"
	void oneStep(uint8_t direction=1);
private:
	uint8_t _puls; //puls control pin of the driver
	
	uint8_t _direction; //direction control pin of the driver
};

Gfaststepper::Gfaststepper(uint8_t puls, uint8_t direction){
	_puls = puls;
	_direction = direction;
	
	pinMode(_puls, OUTPUT);
	pinMode(_direction, OUTPUT);
}

void Gfaststepper::oneStep(uint8_t direction){
	//in order to speed up the excute, ignore the possibility direction use a number not be 1 or 0
	digitalWrite(_direction, direction);
	digitalWrite(_puls, HIGH);
	//Serial.println("Signal pulled high");
	delayMicroseconds(1); //tried works for this driver
	digitalWrite(_puls, LOW);
	//delayMicroseconds(_delayTime); //only one step don't need wait.
}

//------------------------------------------------------------------------------
// PIN defination
//------------------------------------------------------------------------------
#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          3
#define ORIG_X_MAX_PIN          2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN         26
#define ORIG_E0_DIR_PIN          28
#define ORIG_E0_ENABLE_PIN       24

#define ORIG_E1_STEP_PIN         36
#define ORIG_E1_DIR_PIN          34
#define ORIG_E1_ENABLE_PIN       30

#define LED_PIN                  13
#define ORIG_FAN_PIN             9
#define PS_ON_PIN                12

#define HEATER_0_PIN             10
#define HEATER_1_PIN             8
#define HEATER_2_PIN             9
#define TEMP_0_PIN               13   // ANALOG NUMBERING
#define TEMP_1_PIN               14   // ANALOG NUMBERING
#define TEMP_2_PIN               15
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,


//------------------------------------------------------------------------------
// Motor defination
//------------------------------------------------------------------------------
Gfaststepper stepperA(ORIG_X_STEP_PIN, ORIG_X_DIR_PIN);
Gfaststepper stepperB(ORIG_Y_STEP_PIN, ORIG_Y_DIR_PIN);
Gfaststepper stepperC(ORIG_Z_STEP_PIN, ORIG_Z_DIR_PIN);


//------------------------------------------------------------------------------
// Physical parameter defination
//------------------------------------------------------------------------------
/*
                 
                 
                /|\ Y
                 |
    # <<-B       |             # <<-A
      \          |          /
        \        |        /
          \      |      /
            \    |    /
              \  |  /
                \|/
-----------------o------------------> X
                 |
                 |   
                 |      
                 |         
                 |            
                 |            
                 # <<- C      
                 |
                 |
                 
 The line segment AC = AB = BC = 1 meter
 Three motor are mount on a 1 meter high stick.                 
 */
#define DELTA_ALPHA_A 30 // was 330 //unit: degree
#define DELTA_ALPHA_B 150 //was 210
#define DELTA_ALPHA_C 270 //was 90

#define SPOOLDIAMETER 20 //unit:mm

//#define DELTA_Z          2600//1000 //unit: mm, The height of the mount points (motors before), 2800mm high totally, 2600mm actually (a little bit higher)
#define DELTA_SIDE_LONG  5000 // In my Studio, the wall long is about 5 meter (5.175 meter abs)//1000 //unit: mm, AC, AB, BC
#define DELTA_RADIUS     (DELTA_SIDE_LONG / (2 * cos(30 * M_PI / 180))) //unit: mm. line segment OA=OB=OC
#define DELTA_Z  2600
enum axis {X,Y,Z};//X=0, Y=1; Z=2

//the position of each mount point. unit: step
long COORDMOTORA[3];
long COORDMOTORB[3];
long COORDMOTORC[3];

float mode_scale = 1;   // mm or inches? if mm = 1, if inch = 25.4
char mode_name[4] = "mm"; //'in' or 'mm' mode
char absolute_mode=1;  // absolute or incremental programming mode. '1'- abs mode.
int mode_abs = 1;

int robot_uid=0; // robot UID
//GUANG position, unit:mm, datetype float.
float posx = 0; //2014/1/2 huge founden: change here from long to float
float posy = 0; //Here is the coordinate of the end effector.
float posz = 1500; //start point at a hight about 1.5 meter.
float feed_rate = 10; // feed rate is given in units/min and converted to cm/s
long step_delay;

//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------
#define DELAY   (5)
#define BAUD    (115200)
#define MAX_BUF (64)
//#define STEPS_PER_TURN  (3200.0) //200 steps/round x 1/16th mode
//#define MAX_RPM         (200.0) //3.33 round per second
// for arc directions
//#define ARC_CW          (1)
//#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT    1 //was (0.2)
//#define MAX_STEPS_S     (STEPS_PER_TURN*MAX_RPM/60.0)  // steps/s
//#define MIN_VEL         (0.001) // cm/s
// delay between steps, in microseconds.
//#define STEP_DELAY      (5200)  // = 3.5ms, 5200 in Unit MicroSeconds
char buffer[MAX_BUF];
int sofar;
//Motor position, (translated in the length of lines)
// Here store the Old motor positions. If don't have the calibration possibility, the start value should fit to the actual physical start position of the endeffector
//the length of left wire in the "last step".  = l1 (Old) Unit: steps
long laststep1; //now we use laststep1,2,3 for motor A,B,C respectively
long laststep2; 
long laststep3; 

typedef struct {
  long delta; //line length plus direction. (the old & new line difference length)
  unsigned long absdelta; //only length
  int  dir; //line direction. '1'-laststeps get shorter, '0'-laststep get longer.
  long over; //for bresenham's algorithmus
} DLine;

DLine a[3];

/////////////////////////////////////////
///////////  FUNCTION   /////////////////
/////////////////////////////////////////
//translate physical mm based lentgh into steps unit based length.
unsigned long mmToStep(float mm){
  float circumference = 3.14159265359 * (float)SPOOLDIAMETER; //pi * d = circumference
  //Serial.print(F("circumference of spool "));  Serial.println(circumference);  Serial.print(F(""));
  //motor set to full step mode. motor self 200 stp/round. Worm gear transmission ratio 1:25.
  return (mm / circumference * 200 * 25); //'200'-200 steps per round, '25'-wormgear transmission.
}

void Monestep(int j,int dir){
  switch(j){
  case 0:  stepperA.oneStep(dir);  break;
  case 1:  stepperB.oneStep(dir);  break;
  case 2:  stepperC.oneStep(dir);  break;
  }
}

//------------------------------------------------------------------------------
// The new IK for real 3D movements
// Inverse Kinematics - turns XYZ coordinates (unit:mm) into wire lengths lx,ly,lz (unit: steps)
//XYZ is in Unit: mm, l1, l2, l3 is in Unit:step
void IK(float x, float y, float z, long &lx, long &ly, long &lz) {
  long xlong = mmToStep(x);//New target position in step.
  long ylong = mmToStep(y);//it should let us have a faster speed.  
  long zlong = mmToStep(z); 

  float dxA = xlong - COORDMOTORA[X];//The dy, dx, dz are here all minus value actually.
  float dyA = ylong - COORDMOTORA[Y];
  float dzA = zlong - COORDMOTORA[Z];
  lx = sqrt(dxA*dxA + dyA*dyA + dzA*dzA);//The new X line length according to new coordinate.unit: step.
  float dxB = xlong - COORDMOTORB[X];
  float dyB = ylong - COORDMOTORB[Y];
  float dzB = zlong - COORDMOTORB[Z];
  ly = sqrt(dxB*dxB + dyB*dyB + dzB*dzB);
  float dxC = xlong - COORDMOTORC[X];
  float dyC = ylong - COORDMOTORC[Y];
  float dzC = zlong - COORDMOTORC[Z];
  lz = sqrt(dxC*dxC + dyC*dyC + dzC*dzC);
  //Serial.println("lx:" + String(lx)); 
  //Serial.println("ly:" + String(ly));  
  //Serial.println("lz:" + String(lz));  
}

//------------------------------------------------------------------------------
// The new function that move the endeffector in real 3D space.
// this function is using Bresenham algorithmus
void line(float x,float y,float z) {
  //Serial.println("Jump in line() function");

  long l1,l2,l3;
  IK(x,y,z,l1,l2,l3);
  
  a[0].delta = l1 - laststep1;
  a[1].delta = l2 - laststep2;
  a[2].delta = l3 - laststep3;

  long maxsteps = 0;
  for(int i=0; i<3; ++i){
    a[i].absdelta = abs(a[i].delta);
    a[i].dir = a[i].delta > 0 ? 0 : 1; //'0'-laststep get longer, '1'-laststep get shorter.
    a[i].over=0;
    if(maxsteps < a[i].absdelta) maxsteps = a[i].absdelta;
  }  

  for( int i=0; i<maxsteps; ++i){
    for(int j=0; j<3; ++j){
      a[j].over += a[j].absdelta;
      if(a[j].over >= maxsteps){
        a[j].over -= maxsteps;
        Monestep(j,a[j].dir);
      }
    }
    delay(1);
    delayMicroseconds(100);
  }

  laststep1=l1;
  laststep2=l2;
  laststep3=l3;
  posx=x;
  posy=y;
  posz=z;
}

void PRINT(long toP){
  Serial.print(toP);
}

void PRINTLN(long toP){
  Serial.println(toP);
}

/////////////////////////////////////////
////////////   SETUP   //////////////////
/////////////////////////////////////////
void setup(){
  pinMode(ORIG_X_STEP_PIN,OUTPUT);
  pinMode(ORIG_Y_STEP_PIN,OUTPUT);
  pinMode(ORIG_Z_STEP_PIN,OUTPUT);  
  pinMode(ORIG_X_DIR_PIN,OUTPUT);  
  pinMode(ORIG_Y_DIR_PIN,OUTPUT);  
  pinMode(ORIG_Z_DIR_PIN,OUTPUT);    
  pinMode(ORIG_X_ENABLE_PIN,OUTPUT);
  pinMode(ORIG_Y_ENABLE_PIN,OUTPUT);
  pinMode(ORIG_Z_ENABLE_PIN,OUTPUT);  
  pinMode(ORIG_X_MIN_PIN,INPUT);
  pinMode(ORIG_Y_MIN_PIN,INPUT);
  pinMode(ORIG_Z_MIN_PIN,INPUT);  
  pinMode(ORIG_X_MAX_PIN,INPUT);
  pinMode(ORIG_Y_MAX_PIN,INPUT);
  pinMode(ORIG_Z_MAX_PIN,INPUT); 
  pinMode(HEATER_0_PIN,OUTPUT); 
  digitalWrite(ORIG_X_ENABLE_PIN, LOW); //enable all motor.
  digitalWrite(ORIG_Y_ENABLE_PIN, LOW);
  digitalWrite(ORIG_Z_ENABLE_PIN, LOW);
  digitalWrite(HEATER_0_PIN,HIGH);
  
  Serial.begin(BAUD);
  Serial.println(F("\n\nHello WORLD! This is GUANG"));

  //init the coordination of each motor. unit: steps
  COORDMOTORA[X] = cos(DELTA_ALPHA_A * M_PI / 180) * mmToStep(DELTA_RADIUS);
  COORDMOTORA[Y] = sin(DELTA_ALPHA_A * M_PI / 180) * mmToStep(DELTA_RADIUS);
  COORDMOTORB[X] = cos(DELTA_ALPHA_B * M_PI / 180) * mmToStep(DELTA_RADIUS);
  COORDMOTORB[Y] = sin(DELTA_ALPHA_B * M_PI / 180) * mmToStep(DELTA_RADIUS);
  COORDMOTORC[X] = cos(DELTA_ALPHA_C * M_PI / 180) * mmToStep(DELTA_RADIUS);
  COORDMOTORC[Y] = sin(DELTA_ALPHA_C * M_PI / 180) * mmToStep(DELTA_RADIUS);
  COORDMOTORA[Z] = COORDMOTORB[Z] = COORDMOTORC[Z] = mmToStep(DELTA_Z);

  IK(0,0,1500,laststep1,laststep2,laststep3);  //initial start point (0,0,1500) unit:mm

  sofar=0;   // initialize the read buffer

  //setFeedRate(MAX_VEL*30/mode_scale);  // *30 because i also /2
  //setFeedRate(2000); //start at a lower speed.
  Serial.println("laststep1:" + String(laststep1));
  Serial.println("laststep2:" + String(laststep2));
  Serial.println("laststep3:" + String(laststep3));
  Serial.println("COORDMOTORA[X]:" + String(COORDMOTORA[X]));
  Serial.println("COORDMOTORA[Y]:" + String(COORDMOTORA[Y]));
  Serial.println("COORDMOTORB[X]:" + String(COORDMOTORB[X]));
  Serial.println("COORDMOTORB[Y]:" + String(COORDMOTORB[Y]));
  Serial.println("COORDMOTORC[X]:" + String(COORDMOTORC[X]));
  Serial.println("COORDMOTORC[Y]:" + String(COORDMOTORC[Y]));
  Serial.println("COORDMOTORA[z]=COORDMOTORB[z]=COORDMOTORC[z]: " + String(COORDMOTORC[Z]));
  Serial.print(F("DELTA_RADIUS in mm:"));  Serial.println(DELTA_RADIUS);   
  Serial.print(F("DELTA_RADIUS in steps:")); long hello = mmToStep(DELTA_RADIUS); Serial.println(hello);
  Serial.println(F("\ninput value mm based as default"));
  Serial.println("> ");
}

/////////////////////////////////////////
/////////////   LOOP   //////////////////
/////////////////////////////////////////
void loop(){  
  while(Serial.available() > 0) {   // listen for serial commands
    buffer[sofar++]=Serial.read();
    if(buffer[sofar-1]==';') break;  // in case there are multiple instructions
  }

  // if we hit a semi-colon, assume end of instruction.
  if(sofar>0 && buffer[sofar-1]==';') {
    // what if message fails/garbled?

    // echo confirmation
    buffer[sofar]=0;
    Serial.println(buffer);

    processCommand();     // do something with the command

    sofar=0;     // reset the buffer
    Serial.println("> ");     // echo completion
  }  
  
  /* check the endstop
  if(digitalRead(ORIG_X_MIN_PIN)==0){
    Serial.println("Estop X pressed");
  }
  if(digitalRead(ORIG_Y_MIN_PIN)==0){
    Serial.println("Estop Y pressed");
  }
  if(digitalRead(ORIG_Z_MIN_PIN)==0){
    Serial.println("Estop Z pressed");
  } 
  */ 
  
}



