/****************************************************************************
 Module
   RobotFSM.c

 Revision
   1.0

 Description
   This is a top level state machine for controlling the behavior of 
	 the ME218B Robot.

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "RobotFSM.h"
#include "DriveSM.h"
#include "BeaconDetect.h"
#include "spudFSM.h"
#include "CorporateManagement.h"
#include "ADMulti.h"
#include "Planner.h"

#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "bitdefs.h"
#include <stdint.h>
#include "inc/hw_memmap.h"

/*----------------------------- Module Defines ----------------------------*/
// Additional time to move towards miner after recognizing its attachment
#define SNUGGLE_TIME 400

// Distance to back up (in mm) after depositing miner
#define REVERSE_ON_DEPOSIT 300
/*---------------------------- Module Functions ---------------------------*/
static void ElectromagnetON(void);
static void ElectromagnetOFF(void);
static void StopAllTimers(void);
/*---------------------------- Module Variables ---------------------------*/
// Keep track of our robot state
static RobotState_t CurrentState;

// Our corporation
static Corporation_t OurCorporation;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

static bool minerDeposited=false;

// Store our planner location
static uint8_t LastPursuedLocation;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitRobotFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority and sets up the initial transition.

****************************************************************************/
bool InitRobotFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial waiting state
  CurrentState = WaitingToStart;
  // Initialize AD system for miner detection event checker
  ADC_MultiInit(1);
  
  //Now initialize port E for electromagnet pin
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
  //Wait until clock is ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R4) != SYSCTL_PRGPIO_R4)
  {
  }
  // Assign as digital pins
  HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= BIT1HI;
  // Set pin as outputs
  HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) |= BIT1HI;
  // Set pins low to start
  HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT1LO;
  
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
     PostRobotFSM

 Parameters
     EF_Event_t ThisEvent, the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue

****************************************************************************/
bool PostRobotFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunRobotFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Set the overall current action of the robot
****************************************************************************/
ES_Event_t RunRobotFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  if (ThisEvent.EventType == GAMESTATE_CHANGE
    && ThisEvent.EventParam == PERMITS_EXPIRED) {
      printf("Permits expired, game has ended.\r\n");
    // End game
      ImmediateStop();
      CurrentState = WaitingToStart;
      StopAllTimers();
  }
    
  switch (CurrentState)
  {
    case WaitingToStart:
    {
      if (ThisEvent.EventType == CORPORATION_SET)
      {
        // Store corporation
        OurCorporation = (Corporation_t) ThisEvent.EventParam;
      } else if (ThisEvent.EventType == GAMESTATE_CHANGE
        && ThisEvent.EventParam == PERMITS_ISSUED)
      {
        // Start game
        printf("Game has started!\r\n");
        // Change state to searching for Miner
        CurrentState = SearchingForMiner;
        // Post event to start searching for beacon
        ES_Event_t NewEvent;
        NewEvent.EventType = START_BEACON_DETECT;
        // Arbitrarily search for MINER 1 of our corporation
        if (OurCorporation == Corp1) {
          NewEvent.EventParam = CKH1;
        } else if (OurCorporation == Corp2) {
          NewEvent.EventParam = GHI1;
        }
        
        PostBeaconDetect(NewEvent);
        break;
      }
    }

    case SearchingForMiner:
    {
      // Approach beacon once detected
      if (ThisEvent.EventType == BEACON_FOUND)
      {
        printf("Beacon found! Headed to Miner.\r\n");
        // Set current state to approaching miner
        CurrentState = ApproachingMiner;
        // Turn on electromagnet
        ElectromagnetON();
        break;
      } else if (ThisEvent.EventType == BEACON_LOST){
        // Rotate fully
        
      }
    }
    
    case ApproachingMiner:
    {
      if (ThisEvent.EventType == MINER_ATTACHED) {
        // Continue to advance forward for 500ms
        ES_Timer_InitTimer(SNUGGLE_TIMER, SNUGGLE_TIME);
      } else if (ThisEvent.EventType == ES_TIMEOUT
        && ThisEvent.EventParam == SNUGGLE_TIMER) {
        printf("MINER attached! Moving to mining region.\r\n");
        
        // Tell Beacon to stop searching
        ES_Event_t StopSearchingEvent;
        StopSearchingEvent.EventType = END_BEACON_DETECT;
        PostBeaconDetect(StopSearchingEvent);
        
        // Set up planner
        // Send permit location as destination
        uint8_t Destination = WhereShouldWeGo();
        SetPlannerDest(Destination);
        LastPursuedLocation = Destination;
        // Set miner location as current location
        uint8_t CurrentLocation = QueryCurrentLocation();
        SetPlannerStart(CurrentLocation);
        // Post run event to planner
        ES_Event_t NewEvent;
        NewEvent.EventType = RUN_PLANNER;
        PostPlanner(NewEvent);
        // Update state to MovingMiner
        CurrentState = MovingMiner;
      }else if (ThisEvent.EventType == BEACON_LOST){

        // Lost miner, going back into search state
        // Prepares us to handle BEACON_FOUND event
        CurrentState = SearchingForMiner;
        // printf("Approaching to SearchingFor Miner state\r\n");        
      }
      
      break;
    }
    
    case MovingMiner: {
      if (ThisEvent.EventType == MINER_LOCATION) {
        // Continue for a brief moment
        ES_Timer_InitTimer(SNUGGLE_TIMER, SNUGGLE_TIME);
      }
      else if (ThisEvent.EventType == ES_TIMEOUT
        && ThisEvent.EventParam == SNUGGLE_TIMER) {
         // New location detected for our miner
        uint8_t CurrentLocation = QueryCurrentLocation();
        // Check our permitting location
        uint8_t Destination = WhereShouldWeGo();
        // If they match, stop! We're mining!
        if (CurrentLocation == LastPursuedLocation) {
          printf("Miner is on permitted region. Yahoo!\r\n");
          // Stop and back up
          ImmediateStop();
          ElectromagnetOFF();
          
          //Post a miner deposited flag
          minerDeposited = true;
          
          ES_Event_t RevEvent;
          RevEvent.EventType = MoveRevDist;
          RevEvent.EventParam = REVERSE_ON_DEPOSIT;
          PostDriveSM(RevEvent);
        } else {
          printf("New miner location found! Rerunning planner.\r\n");
          // Set up planner
          // Send permit location as destination
          SetPlannerDest(Destination);
          LastPursuedLocation = Destination;
          // Set miner location as current location
          SetPlannerStart(CurrentLocation);
          // Post run event to planner
          ES_Event_t NewEvent;
          NewEvent.EventType = RUN_PLANNER;
          PostPlanner(NewEvent);
          }
      }
      else if (ThisEvent.EventType == MINER_DETACHED){
        printf("Moving to SearchingFor Miner\r\n");
        // Begin Searching again
        CurrentState = SearchingForMiner;
        // Post event to start searching for beacon
        ES_Event_t NewEvent;
        NewEvent.EventType = START_BEACON_DETECT;
        // Arbitrarily search for MINER 1 of our corporation
        if (OurCorporation == Corp1) {
          NewEvent.EventParam = CKH1;
        } else if (OurCorporation == Corp2) {
          NewEvent.EventParam = GHI1;
        }
        PostBeaconDetect(NewEvent);
      }
      break;
    }
		
    default:
      break;
  }                                   // end switch on Current State
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryRobotSM

 Parameters
     None

 Returns
     RobotState_t The current state of the Robot state machine

 Description
     returns the current state of the robot state machine

****************************************************************************/
RobotState_t QueryRobotSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
     ElectromagnetON

 Parameters
     None

 Returns
     None

 Description
     Turns on electromagnet

****************************************************************************/
void ElectromagnetON(void)
{
  // Set pin high
  HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT1HI;
}

/****************************************************************************
 Function
     ElectromagnetOFF

 Parameters
     None

 Returns
     None

 Description
     Turns off electromagnet

****************************************************************************/
void ElectromagnetOFF(void)
{
  // Set pin low
  HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT1LO;
}


/****************************************************************************
 Function
    StopAllTimers

 Parameters
   None

 Returns
   None

 Description
   Stop all timers, individually
****************************************************************************/
void StopAllTimers(void) {
  // Stop all timers manually
  ES_Timer_StopTimer(BEACON_TIMER);
  ES_Timer_StopTimer(ACCEL_TIMER);
  ES_Timer_StopTimer(MEASURE_TIMER);
  ES_Timer_StopTimer(ACCEL_FLAG);
  ES_Timer_StopTimer(SNUGGLE_TIMER);
  ES_Timer_StopTimer(COLLISION_TIMER);
  ES_Timer_StopTimer(STALL_TIMER);
  ES_Timer_StopTimer(BEACON_NOT_FOUND_TIMER);
  ES_Timer_StopTimer(BUMP_MONITOR);
}

/****************************************************************************
 Function
     setMinerDeposited

 Parameters
     bool

 Returns
     void

 Description
     handles requests to set miner deposited flag

****************************************************************************/
void setMinerDeposited(bool flagSet){
  minerDeposited = flagSet;
}

/****************************************************************************
 Function
     getMinerDeposited

 Parameters
     None

 Returns
     bool

 Description
     handles queries for the flag that determines whether a miner was recently deposited

****************************************************************************/
bool getMinerDeposited(void){
  return minerDeposited;
}

