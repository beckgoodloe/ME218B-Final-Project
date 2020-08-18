/****************************************************************************
 Module
   AccelService.c

 Revision
   1.0

 Description
   This is a file for implementing an accelerometer control service under the
   Gen2 Events and Services Framework.

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "AccelService.h"
#include "AccelSerial.h"
#include <math.h>
#include "DriveSM.h"
#include "EventCheckers.h"

/*----------------------------- Module Defines ----------------------------*/
// Query the accelerometer at 20Hz
#define ACCEL_QUERY_RATE 50

// Important accelerometer registers
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAAX0 0x32

// Important commands
#define SET_BW 0x09
#define SET_POWER BIT3HI
#define SET_FORMAT 0x00
#define EMPTY_QUERY 0x00

// Mask to set read bit and multi write
#define READ_MASK 0xC0

// Orientation constants
#define ACCEL_AT_NORTH 10.0
#define PI 3.14159

// Bump detect constants
#define BUMP_THRESHOLD 50

#define ACCEL_FLAG_DELAY 400
#define RAMP_UP_CONSTANT 20

#define BUMP_MONITOR_THRESHOLD 10000

/*---------------------------- Module Functions ---------------------------*/
float clamp(float num, float min, float max);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

static uint16_t maxX=0;
static uint16_t maxY=0;
static bool rampOver = false; //boolean to track the ramp up of the accelerometer and ignore initial values
static uint16_t debugCounter = 0; //ignoring initial values of accelerometer
static bool accelFlag = false; //Determines whether program is checking for bumps: False --> checking
static bool multipleCollisionFlag = false; //Determines whether or not a collision counting timer is live

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitAccelService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and intializes SPI1 for accelerometer
 
****************************************************************************/
bool InitAccelService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  // Intialize serial library
  InitAccelSerial();
  
  // Set measure mode
  TransmitAccelSerial(POWER_CTL);
  TransmitAccelSerial(SET_POWER);
  
  // Start timer to periodically query accelerometer
  ES_Timer_InitTimer(ACCEL_TIMER, ACCEL_QUERY_RATE);
  
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostAccelService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostAccelService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunAccelService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Queries the accelerometer via serial. Posts the results.

****************************************************************************/
ES_Event_t RunAccelService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (ThisEvent.EventType) {
    case ES_TIMEOUT: {
      if (ThisEvent.EventParam == ACCEL_TIMER) {
        // Query accelerometer. We want to read the AX0 reg and following 3
        // registers, so send the reg command and 4 empty queries
        TransmitAccelSerial(READ_MASK | DATAAX0);
        TransmitAccelSerial(EMPTY_QUERY);
        TransmitAccelSerial(EMPTY_QUERY);
        TransmitAccelSerial(EMPTY_QUERY);
        TransmitAccelSerial(EMPTY_QUERY);
        // Re-intialize timer
        ES_Timer_InitTimer(ACCEL_TIMER, ACCEL_QUERY_RATE);
      }
      break;
    }
    
    case BYTE_RECEIVED: {
      detectCollision();
    }
    
    default:
      break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
   QueryHeading

 Parameters
   None

 Returns
    double: thetaY, heading in degrees relative to north calculated from
            y acceleration

 Description
   Calculates and returns thetaY, heading in degrees relative to north

****************************************************************************/
double QueryHeading(void) {
  // Get most recent acceleration values
  int16_t xAccel = GetxAccel();
  int16_t yAccel = GetyAccel();
  // Calculate estimated theta from y acceleration
  double thetaY = 180/PI*acos(clamp(yAccel/ACCEL_AT_NORTH, -1, 1));
  // Use x acceleration to complete the circle
  if (xAccel < 0 && thetaY != 0) {
    thetaY = 360 - thetaY;
  }
  
  return thetaY;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
     clamp

 Parameters
     num (float) - number of concern
     max (float) - upper limit for num
     min (float) - lower limit for num

 Returns
     num (float)

 Description
     Sets and returns num within bounds of min (floor) and max (ceiling).

****************************************************************************/
float clamp(float num, float min, float max) {
  // If num exceeds max, apply ceiling
  if (num > max) {
    num = max;
  }
  // If num is below min, apply floor
  if (num < min) {
    num = min;
  }
  return num;
}

/****************************************************************************
 Function
     Retrieve max x acceleration

 Parameters
     void

 Returns
     num (uint16_t)

 Description
     Allows external modules to query the maximum x acceleration seen since start up

****************************************************************************/
uint16_t getMaxX(void){
  return maxX;
}

/****************************************************************************
 Function
     Retrieve max y acceleration

 Parameters
     void

 Returns
     num (uint16_t)

 Description
     Allows external modules to query the maximum y acceleration seen since start up

****************************************************************************/
uint16_t getMaxY(void){
  return maxY;
}

/****************************************************************************
 Function
     Sets flag to prevent bump detection from triggering

 Parameters
     flagSet (bool) -- mode the flag is to be set to

 Returns
     void

 Description
     Setting this flag as true will stop bumps from being detected until flag is unset.
     Setting this flag as false will allow bump detection to proceed.

****************************************************************************/
void setAccelFlag(bool flagSet){
  // Set flag to desired value
  accelFlag = flagSet;
}

/****************************************************************************
 Function
     Detect collisions

 Parameters
     void

 Returns
     void

 Description
     At each iteration of the accelerometer update, this function checks to see if
     the instantaneous raw value is more than a threshold value above the moving average.
     If it is, a bump is detected and an event is posted to DriveSM.

****************************************************************************/
void detectCollision(){
  
  //Ramp up counter to avoid initial misreadings from accelerometer during initialization
  if(debugCounter>RAMP_UP_CONSTANT){
     rampOver = true;
   }
  //If the ramp up period is over and there isnt a flag
   if(rampOver && !accelFlag){
     //check thresholds
     if(!getMinerAttached()){
       if(abs(GetRawyAccel()-GetyAccel())>BUMP_THRESHOLD){
       //Print to TeraTerm for debug
       printf("Bump Detected\r\n");

       //Make sure the timer isn't already started
       if(!getMultipleCollisionFlag()){
         //Start bump monitor timer
         ES_Timer_InitTimer(BUMP_MONITOR,BUMP_MONITOR_THRESHOLD);
         //Set the bump monitor flag
         setMultipleCollisionFlag(true);
         //Reset DriveSM collision count
         ResetCollisionCount();
       }
       
       //Reset debounce flag and start timer
       setAccelFlag(true);
       ES_Timer_InitTimer(ACCEL_FLAG, ACCEL_FLAG_DELAY);
       
       //Respond to collision
       ES_Event_t ThisEvent1;
       ES_Event_t ThisEvent2;
       ThisEvent1.EventType = EV_FRONT_LIMIT_HIT;
       ThisEvent2.EventType = EV_BACK_LIMIT_HIT;
       PostDriveSM(ThisEvent1);
       PostDriveSM(ThisEvent2);
     }
    }else{
      if(abs(GetRawyAccel()-GetyAccel())>(.5*BUMP_THRESHOLD)){
       //Print to TeraTerm for debug
       printf("Bump Detected\r\n");

       //Make sure the timer isn't already started
       if(!getMultipleCollisionFlag()){
         //Start bump monitor timer
         ES_Timer_InitTimer(BUMP_MONITOR,BUMP_MONITOR_THRESHOLD);
         //Set the bump monitor flag
         setMultipleCollisionFlag(true);
         //Reset DriveSM collision count
         ResetCollisionCount();
       }
       
       //Reset debounce flag and start timer
       setAccelFlag(true);
       ES_Timer_InitTimer(ACCEL_FLAG, ACCEL_FLAG_DELAY);
       
       //Respond to collision
       ES_Event_t ThisEvent1;
       ES_Event_t ThisEvent2;
       ThisEvent1.EventType = EV_FRONT_LIMIT_HIT;
       ThisEvent2.EventType = EV_BACK_LIMIT_HIT;
       PostDriveSM(ThisEvent1);
       PostDriveSM(ThisEvent2);
     }
    }
  }else{
     debugCounter++;
    }
}

/****************************************************************************
 Function
     Sets flag to indicate whether a multiple collision timer is counting

 Parameters
     flagSet (bool) -- mode the flag is to be set to

 Returns
     void

 Description
     Setting this flag as true will stop bumps from being detected until flag is unset.
     Setting this flag as false will allow bump detection to proceed.

****************************************************************************/
void setMultipleCollisionFlag(bool flagSet){
  multipleCollisionFlag = flagSet;
}

/****************************************************************************
 Function
     Retrieve multiple collision flag

 Parameters
     void

 Returns
     bool

 Description
     Allows external modules to query whether the multiple collision timer is live

****************************************************************************/
bool getMultipleCollisionFlag(void){
  return multipleCollisionFlag;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/