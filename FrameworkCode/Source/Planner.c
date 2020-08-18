/****************************************************************************
 Module
   Planner.c

 Revision
   1.0

 Description
   This is a file for implementing a motion planning service under the
   Gen2 Events and Services Framework.

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "Planner.h"
#include "DriveSM.h"
#include <math.h>
#include "AccelService.h"

/*----------------------------- Module Defines ----------------------------*/
//squares on each side of board
#define SIZE_OF_BOARD 4

// Time for a heading measurement to stabilize
#define MEASUREMENT_TIME 1000
/*---------------------------- Module Functions ---------------------------*/
static uint16_t PlanPath(uint8_t start, uint8_t dest);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

// Store our most recent location update and our destination
static uint8_t Destination;
static uint8_t CurrentLocation;

// Store whether we are currently rotating
static bool CurrentlyRotating;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitPlanner

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority.

****************************************************************************/
bool InitPlanner(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  // Initialize rotating bool
  CurrentlyRotating = false;
  
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
     PostPlanner

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
bool PostPlanner(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunPlanner

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Orients bot to face destination, move toward it.
****************************************************************************/
ES_Event_t RunPlanner(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType) {
    case RUN_PLANNER: {
      printf("Running planner! Let's stop the bot and measure.\r\n");
      // Stop bot for measurement
      ImmediateStop();
      // Wait for angle measurement to stabilize
      ES_Timer_InitTimer(MEASURE_TIMER, MEASUREMENT_TIME);
      break;
    }
    
    case ES_TIMEOUT: {
      if (ThisEvent.EventParam == MEASURE_TIMER) {
        printf("Time to measure!\r\n");
        // Measure heading
        double CurrentHeading = QueryHeading();
        // Calculate desired heading
        double DesiredHeading = PlanPath(CurrentLocation, Destination);
        // Calculate difference in heading, to rotate
        double HeadingDiff = DesiredHeading - CurrentHeading;
        // Rotate to desired heading
        printf("We're at square %u, and headed to %u\r\n", CurrentLocation, Destination);
        printf("Our heading is %f, we want to be at %f, so we rotate %f\r\n", CurrentHeading, DesiredHeading, HeadingDiff);
        ES_Event_t NewEvent;
        if (HeadingDiff > 0) {
          NewEvent.EventType = RotateCCWAngle;
        } else {
          NewEvent.EventType = RotateCWAngle;
        }
        NewEvent.EventParam = (uint16_t) abs(HeadingDiff);
        PostDriveSM(NewEvent);
        // Store that we are currently rotating
        CurrentlyRotating = true;
      }
      break;
    }
    
    case EV_ROTATION_COMPLETED: {
      if (CurrentlyRotating) {
        CurrentlyRotating = false;
        // Drive forward
        printf("Woot! We've rotated! Headed home.\r\n");
        ES_Event_t FwdEvent;
        FwdEvent.EventType = MoveFwd;
        PostDriveSM(FwdEvent);
      }
      break;
    }
    
    default:
      break;
  }    
  
  return ReturnEvent;
}

/****************************************************************************
 Function
    SetPlannerDest

 Parameters
   uint8_t location code of desired destination

 Returns
   None

 Description
   Stores desired destination
****************************************************************************/
void SetPlannerDest(uint8_t Location) {
  Destination = Location;
}

/****************************************************************************
 Function
    SetPlannerStart

 Parameters
   uint8_t location code of current location

 Returns
   None

 Description
   Stores current location
****************************************************************************/
void SetPlannerStart(uint8_t Location) {
  CurrentLocation = Location;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
    PlanPath

 Parameters
   uint8_t location code of current location
   uint8_t location code of destination location

 Returns
   uint16_t absolute heading to orient robot to move to dest

 Description
   Calculates heading required to move from current square to
   destination square.
****************************************************************************/
uint16_t PlanPath(uint8_t start, uint8_t dest){
    //calculate row column index
    double r1 = floor(start/SIZE_OF_BOARD);
    double c1 = start%(SIZE_OF_BOARD);
    double r2 = floor(dest/SIZE_OF_BOARD);
    double c2 = dest%SIZE_OF_BOARD;
   
    //cover horizontal edge cases
    if(r1==r2){
        if(c2>c1){
            return 270;
        }else{
            return 90;
        }
    }
   
    //cover vertical edge cases
    if(c1==c2){
        if(r2>r1){
            return 180;
        }else{
            return 0;
        }
    }
   
    //simplifying arguments for calculation
    double r_diff = r2-r1;
    double c_diff = c2-c1;
    float argument = c_diff/r_diff;
   
    //calculate angle
    float phi = (180/3.14159)*atan(argument);
   
    //adjust based on orientation
    if( r_diff>0 && c_diff>0) {
        return phi + 180;
    } else if(r_diff<0 && c_diff>0) {
        return phi + 360;
    } else if(r_diff>0 && c_diff<0) {
        return phi + 180;
    }
    return phi;
}

/****************************************************************************
 Function
    Distance

 Parameters
   uint8_t location code of current location
   uint8_t location code of destination location

 Returns
   uint8_t unitless distance for current location to destination for comparison

 Description
   Calculates distance required to move from current square to
   destination square.
****************************************************************************/
uint8_t Distance(uint8_t start, uint8_t dest){
    //calculate row column index
    uint8_t r1 = floor(start/SIZE_OF_BOARD);
    uint8_t c1 = start%(SIZE_OF_BOARD);
    uint8_t r2 = floor(dest/SIZE_OF_BOARD);
    uint8_t c2 = dest%SIZE_OF_BOARD;
    
    return (c2-c1)*(c2-c1) + (r2-r1)*(r2-r1);
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

