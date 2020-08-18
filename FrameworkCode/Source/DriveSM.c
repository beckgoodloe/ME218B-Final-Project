/****************************************************************************
 Module
   DriveSM.c

 Revision
   1.0.1

 Description
   This is a file for implementing the drive state machine under the
   Gen2 Events and Services Framework.
   This handles things like moving forward, backward, and rotating
   It can handle moving indefinitely or for specific distances with the
   inclusion of the encoder library

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "DriveSM.h"
#include "MotorControl.h"
#include "EncoderModule.h"
#include "Planner.h"
#include "AccelService.h"
#include "RobotFSM.h"
#include "BeaconDetect.h"

/*----------------------------- Module Defines ----------------------------*/
#define COLLISION_TIME  1000
#define COLLISION_MAX 2

#define ROTATE_DC 50

#define ACCEL_FLAG_DELAY 700

#define STUCK_ANGLE_RESPONSE 180
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

void ResetCollisionCount(void);
void SetWheelDC(uint8_t DC);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static DriveState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

static uint8_t CollisionCount = 0;
static uint8_t DriveDC = HALF_SPEED_DC;

static bool beaconNotFoundFlag = false;
static bool ManyManyCollisions = false;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDriveSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitDriveSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPsuedoState;
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
     PostDriveSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostDriveSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunDriveSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunDriveSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  //Setting debounce flag for accelerometer bump detection
  if(ThisEvent.EventParam != ACCEL_FLAG){
    setAccelFlag(true);
    ES_Timer_InitTimer(ACCEL_FLAG, ACCEL_FLAG_DELAY);
  }
  
  switch(ThisEvent.EventType)
  {
    case ES_INIT:
    {
      if(CurrentState == InitPsuedoState)
      {
        // Initialize motor control (start PWM and configure direction pins)
        InitMotorControl();
        InitEncoderCapture();
        InitPeriodicInt();
                
        // Initialize variables
        SetWheelDC(STOP_DC); // start in stopped state
        
        // Zero and disable related encoder functions
        ResetEncoderCount();
        IncrementEncoderCount(); // start in normal encoder counting mode
        DisablePositionTracking();

        // Stop Motors
        StopMotors();
        // now put the machine into the actual initial state
        CurrentState = Stopped;
      }
    break;
    }
    
    case EV_POSITION_REACHED:
    {
      if (CurrentState == DrivingFwdDist || CurrentState == DrivingRevDist)
      {
        // post straight completion
        ES_Event_t Event2Post;
        Event2Post.EventType = EV_STRAIGHT_COMPLETED;
        ES_PostList00(Event2Post);
        
        if(beaconNotFoundFlag){
          //Reset the flag to be false
          beaconNotFoundFlag = false;
          //Move back into beacon detect state
          ES_Event_t RedetectEvent;
          RedetectEvent.EventType = RESUME_BEACON_DETECT;
          ES_PostList00(RedetectEvent);
        }
        
        if(getMinerDeposited()){          
          //Reset miner deposited flag
          setMinerDeposited(false);
          
          //Move back into miner detection state
          ES_Event_t RedetectEvent;
          RedetectEvent.EventType = START_BEACON_DETECT;
          PostBeaconDetect(RedetectEvent);
        }
        
        if(ManyManyCollisions){
          //clear collision flag
          ManyManyCollisions = false;
          
          //Place us back in beacon search
          ES_Event_t NewEvent;
          NewEvent.EventType = RESUME_BEACON_DETECT;
          PostBeaconDetect(NewEvent);
        }
      } 
      else if (CurrentState == RotatingCWAngle || CurrentState == RotatingCCWAngle)
      {
        // post rotation completion
        ES_Event_t Event2Post;
        Event2Post.EventType = EV_ROTATION_COMPLETED;
        printf("Done rotating\r\n");

        ES_PostList00(Event2Post); 
        PostPlanner(Event2Post);
      } 
      ImmediateStop(); // Done moving, so update this SM
      if(ManyManyCollisions){
                
        //Clear collision counter
        ResetCollisionCount();
        
        //MOve forward out of collision region
        ES_Event_t FwdEvent;
        FwdEvent.EventType = MoveFwdDist;
        FwdEvent.EventParam = 300;
        PostDriveSM(FwdEvent);
      }
      break; 
    }
    
    case Stop:
    {
      ResetEncoderCount();
      DisablePositionTracking();
      SetWheelDC(STOP_DC);
      StopMotors();
      CurrentState = Stopped; 
      break;
    }
    
    case MoveFwd: // Move forward indefinitely
    {
      ResetEncoderCount();
      DisablePositionTracking();
      CurrentState = DrivingFwd;
      SetWheelDC(HALF_SPEED_DC);
      DriveStraight(FWD);
      break;
    }
    
    case MoveFwdDist: // Move forward a finite distance in millimeters
    {
      ResetEncoderCount();
      SetEncoderDistanceMM(ThisEvent.EventParam);
      SetWheelDC(HALF_SPEED_DC);
      DriveStraight(FWD);
      CurrentState = DrivingFwdDist;
      break;
    }
    
    case MoveRev:
    {
      DisablePositionTracking();
      ResetEncoderCount();
      SetWheelDC(DriveDC);
      DriveStraight(REV);
      CurrentState = DrivingRev;
      break;
    }
    
    case MoveRevDist:
    {
      ResetEncoderCount();
      // Set Distance
      SetEncoderDistanceMM(ThisEvent.EventParam);
      SetWheelDC(DriveDC);
      DriveStraight(REV);
      CurrentState = DrivingRevDist;
      break;
    }
    
    case RotateCW:
    {
      ResetEncoderCount();    
      DisablePositionTracking();
      Rotate(CW);
      SetWheelDC(DriveDC);
      CurrentState = RotatingCW;
      break;
    }
    
    case RotateCWAngle:
    {
      ResetEncoderCount();    
      SetEncoderDistanceDeg(ThisEvent.EventParam);
      Rotate(CW);
      SetWheelDC(DriveDC);
      CurrentState = RotatingCWAngle;
      break;
    }
    
    
    case RotateCCW:
    {
      ResetEncoderCount();    
      DisablePositionTracking();
      Rotate(CCW);
      SetWheelDC(DriveDC);
      CurrentState = RotatingCCW;
      break;
    }
    
    case RotateCCWAngle:
    {
      ResetEncoderCount();    
      SetEncoderDistanceDeg(ThisEvent.EventParam);
      Rotate(CCW);
      SetWheelDC(DriveDC);
      CurrentState = RotatingCCWAngle;
      break;
    }
    
    case EV_FRONT_LIMIT_HIT:
    {
      if (CurrentState == DrivingFwd || CurrentState == DrivingFwdDist){
        // Oops, we hit it in the direction of motion 
        
        //Increment collision counter
        CollisionCount++;
        
        // Begin driving away from intended final position so decrement encoders
        DecrementEncoderCount();
        
        DriveStraight(REV);
        SetWheelDC(DriveDC);
        
        // Start move away from collision timer
        ES_Timer_InitTimer(COLLISION_TIMER, COLLISION_TIME);
        CurrentState = RevAfterFrontCollision;
      }
      break;
    }
    
    case EV_BACK_LIMIT_HIT:
    {
      if(CurrentState == DrivingRev || CurrentState == DrivingRevDist){
        // Oops, we hit it in direction of motion
        CollisionCount++;
        
        // Begin driving away from intended final position so decrement encoders
        DecrementEncoderCount();
        
        DriveStraight(FWD);
        SetWheelDC(DriveDC);
        
        // Start move away from collision timer
        ES_Timer_InitTimer(COLLISION_TIMER, COLLISION_TIME);
        CurrentState = FwdAfterBackCollision; 
      }
    break;
   }
   
   case EV_STALL:{
    DecrementEncoderCount();
    
    if(CurrentState == RotatingCCW || CurrentState == RotatingCCWAngle){
      //CollisionCount++;
      Rotate(CW);
      SetWheelDC(DriveDC);  
      CurrentState = CWAfterStall;
    } else if(CurrentState == RotatingCW || CurrentState == RotatingCWAngle){
      //CollisionCount++;
      Rotate(CCW);
      SetWheelDC(DriveDC);
      CurrentState = CCWAfterStall;
    }
    // Start move away from collision timer
    ES_Timer_InitTimer(COLLISION_TIMER, COLLISION_TIME);

    break;
   }
   
   case ES_TIMEOUT:
   {
     if(ThisEvent.EventParam == COLLISION_TIMER)
     {
       if(CurrentState == RevAfterFrontCollision)
       {
          // Timeout indicates that we should stop backing up
          // Move Forward again
          IncrementEncoderCount(); 
          DriveStraight(FWD);
          SetWheelDC(DriveDC);
          CurrentState = DrivingFwd;
          ES_Event_t RedetectEvent;
          RedetectEvent.EventType = RESUME_BEACON_DETECT;
          ES_PostList00(RedetectEvent);
       } 
       else if (CurrentState == FwdAfterBackCollision) {
          // Timeout indicates that we should stop backing up
          // Move in reverse again
          IncrementEncoderCount();
          DriveStraight(REV);
          SetWheelDC(DriveDC);
          CurrentState = DrivingRev;
          ES_Event_t RedetectEvent;
          RedetectEvent.EventType = RESUME_BEACON_DETECT;
          ES_PostList00(RedetectEvent);
       }
       else if (CurrentState == CWAfterStall){
          IncrementEncoderCount();
          Rotate(CCW);
          SetWheelDC(DriveDC);
          CurrentState = RotatingCCW;
          ES_Event_t RedetectEvent;
          RedetectEvent.EventType = RESUME_BEACON_DETECT;
          ES_PostList00(RedetectEvent);
       } else if(CurrentState == CCWAfterStall){
          IncrementEncoderCount();
          Rotate(CW);
          SetWheelDC(DriveDC);
          CurrentState = RotatingCW;
          ES_Event_t RedetectEvent;
          RedetectEvent.EventType = RESUME_BEACON_DETECT;
          ES_PostList00(RedetectEvent);
       }
     } else if(ThisEvent.EventParam == ACCEL_FLAG){
       setAccelFlag(false);
     } else if(ThisEvent.EventParam == BUMP_MONITOR){
       setMultipleCollisionFlag(false);

     }else{
       ES_Event_t NewEvent;
       NewEvent.EventType = RESUME_BEACON_DETECT;
       PostBeaconDetect(NewEvent);
     }
     break;
   }
   default: 
   {
   };
  }
  
  if(CollisionCount > COLLISION_MAX){
    ResetCollisionCount();  
    //Set a collision flag
    ManyManyCollisions = true;
    
    ES_Event_t DriveEvent;
    DriveEvent.EventType = RotateCCWAngle;
    DriveEvent.EventParam = STUCK_ANGLE_RESPONSE;
    PostDriveSM(DriveEvent);
  }
   
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryDriveSM

 Parameters
     None

 Returns
     DriveState_t The current state of the Drive state machine

 Description
     returns the current state of the Drive state machine
 Notes


****************************************************************************/
DriveState_t QueryDriveSM(void)
{
  return CurrentState;
}

/****************************************************************************
 Function
     ResetCollisionCount

 Parameters
     None

 Returns
      None

 Description
     Resets value of collision count back to 0
 
 Notes


****************************************************************************/
void ResetCollisionCount(void)
{
  CollisionCount = 0;
}

/****************************************************************************
 Function
     SetWheelDC

 Parameters
     Duty Cycle

 Description
     Sets target duty cycle value for wheel control law
****************************************************************************/
void SetWheelDC(uint8_t DC){
  SetTargetRightDC(DC);
  SetTargetLeftDC(DC);
}

/****************************************************************************
 Function
     SetDriveDC

 Parameters
     New duty cycle

 Description
      Takes value to set as nominal duty cycle for bot motion
****************************************************************************/
void SetDriveDC(uint8_t DC){
  DriveDC = DC;
  
}

void ImmediateStop(void){
  // Always want to stop, if we get a command to stop.
  // Don't necessarily want to post it to the queue, because it will 
  // just pass right through, which we don't necessarily want (need to wait for calculations)
  ResetEncoderCount();
  DisablePositionTracking();
  SetWheelDC(STOP_DC);
  StopMotors();
  CurrentState = Stopped;  
}
/****************************************************************************
 Function
     SetBeaconNotFound

 Parameters
     Flag that determines if we are exiting a beacon not found state

 Description
      Takes flag that controls exiting beacon not found event a posts it.
****************************************************************************/
void SetBeaconNotFoundFlag(bool flagSet){
  beaconNotFoundFlag = flagSet;
  
}

/****************************************************************************
 Function
     SetManyManyCollisions

 Parameters
      Bool

 Description
      Takes flag that controls getting out of a bad place and posts it.
****************************************************************************/
void SetManyManyCollisions(bool flagSet){
  ManyManyCollisions = flagSet;
  
}
/***************************************************************************
 private functions
 ***************************************************************************/
