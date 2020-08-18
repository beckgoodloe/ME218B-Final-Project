/****************************************************************************
 Module
   StallService.c

 Revision
   1.0.1

 Description
   Checks if stalling by checking differences in encoder count

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "StallService.h"
#include "DriveSM.h"
#include "EncoderModule.h"

#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "bitdefs.h"
/*----------------------------- Module Defines ----------------------------*/
#define TICKS_PER_MS  40000
#define STALL_CHECK_TIME  200 // ms
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static int32_t LastLeftEncoderCount;
static int32_t LastRightEncoderCount;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitStallService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Starts the timer necessary to check stall timers
     
****************************************************************************/
bool InitStallService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  
  ES_Timer_InitTimer(STALL_TIMER, STALL_CHECK_TIME);
  LastLeftEncoderCount = GetEncoderCountLeft();
  LastRightEncoderCount = GetEncoderCountRight();
  
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
     PostStallService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostStallService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunStallService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
    Handles timeouts that check for stalling

****************************************************************************/
ES_Event_t RunStallService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  if((ThisEvent.EventType == ES_TIMEOUT) 
    && (ThisEvent.EventParam == STALL_TIMER)){
    
    DriveState_t state = QueryDriveSM();
    int32_t NewLeft = GetEncoderCountLeft();
    int32_t NewRight = GetEncoderCountRight();
      if(state != Stopped){
        // Check Encoder Counts
        if((NewLeft == LastLeftEncoderCount) || (NewRight == LastRightEncoderCount)){
          //Send event, handle it as a collision
          ES_Event_t ThisEvent;
          ThisEvent.EventType = EV_STALL;
          //PostDriveSM(ThisEvent);
          if(state == DrivingFwdDist || state == DrivingFwd){
            ES_Event_t OtherEvent;
            OtherEvent.EventType = EV_FRONT_LIMIT_HIT;
            PostDriveSM(OtherEvent);
          } else if (state == DrivingRevDist || state == DrivingRev){
            ES_Event_t OtherEvent;
            OtherEvent.EventType = EV_BACK_LIMIT_HIT;
            PostDriveSM(OtherEvent);
          } else if (state == RotatingCW || state == RotatingCWAngle){
            PostDriveSM(ThisEvent);
          } else if (state == RotatingCCW || state == RotatingCCWAngle){
            PostDriveSM(ThisEvent);
          }
        }
        
        //Update previous encoder counts
        LastLeftEncoderCount = NewLeft;
        LastRightEncoderCount = NewRight;
      }
      // reinit timer
      ES_Timer_InitTimer(STALL_TIMER, STALL_CHECK_TIME);
  }
  
  return ReturnEvent;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

