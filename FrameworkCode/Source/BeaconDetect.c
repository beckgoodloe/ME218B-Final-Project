/****************************************************************************
 Module
   BeaconDetect.c

 Revision
   1.0

 Description
   This is an implementation of a service to detect and lock on to miner
   beacons

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BeaconDetect.h"
#include "DriveSM.h"
#include "RobotFSM.h"
#include "CorporateManagement.h" // For corporation enum
#include "spudFSM.h"

#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "bitdefs.h"
#include "EventCheckers.h"

/*----------------------------- Module Defines ----------------------------*/
// Check period frequently to see if beacon is found
#define DETECT_RATE 25

// Check for beacon for only this many seconds before trying somewhere else
#define DETECTION_FAILED_TIME  7000

// Check occasionally to make sure beacon is still in sight
#define CONFIRM_RATE 100

// 40,000 ticks per mS assumes a 40Mhz clock
#define TicksPerMS 40000

// 40 ticks per us
#define TicksPerUS 40

// Print the RPM output at 10Hz
#define UPDATE_PERIOD 100

// Store frequencies for each known beacon (in us)
#define CKH1_PERIOD 300
#define CKH2_PERIOD 500
#define GHI1_PERIOD 700
#define GHI2_PERIOD 1100

#define WIGGLE_INCREMENT  3
#define WIGGLE_COUNT_MAX  6
#define ANGLE_CORRECTION  1
#define STRAIGHT_INCREMENT  400
#define SQUARE_DIAG_MM  849

#define NO_BEACON_MOVE_DISTANCE 500

// Name miners
#define MINER_1 1
#define MINER_2 2
/*---------------------------- Module Functions ---------------------------*/
static MinerName_t IsPeriodGood(void);

/*---------------------------- Module Variables ---------------------------*/
// State variable
static BeaconState_t CurrentState;

// Keep track of current corporation
static Corporation_t OurCorporation;

// Keep track of current miner being pursued
static MinerName_t CurrentMiner;

// Keep track of period of current miner
static uint16_t CurrentBeaconPeriod;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// Stores calculated period (ticks between beacon pulse)
static uint32_t Period;

// Stores tick count at last beacon pulse
static uint32_t LastCapture;

//Used to wiggle back and forth when the miner is lost
static int16_t WiggleDegree = 10;
static uint8_t WiggleCounter = 0;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitBeaconDetect

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition, and
     initializes interrupt to time beacon pulses

****************************************************************************/
bool InitBeaconDetect(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the waiting state
  CurrentState = Waiting;
  
  // we will use Timer B in Wide Timer 5 to capture the input
  // start by enabling the clock to the timer (Wide Timer 5)
  HWREG(SYSCTL_RCGCWTIMER) |=SYSCTL_RCGCWTIMER_R5;
  // enable the clock to Port D
  HWREG(SYSCTL_RCGCGPIO) |=SYSCTL_RCGCGPIO_R3;
  // since we added this Port D clock init, we can immediately start
  // into configuring the timer, no need for further delay
  
  // make sure that timer (Timer A) is disabled before configuring
  HWREG(WTIMER5_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
  // set it up in 32bit wide (individual, not concatenated) mode
  // the constant name derives from the 16/32 bit timer, but this is a 32/64
  // bit timer so we are setting the 32bit mode
  HWREG(WTIMER5_BASE+TIMER_O_CFG) =TIMER_CFG_16_BIT;
  
  // we want to use the full 32 bit count, so initialize the Interval Load
  // register to 0xffff.ffff (its default value :-)
  HWREG(WTIMER5_BASE+TIMER_O_TBILR) =0xffffffff;
  // we don't want any prescaler (it is unnecessary with a 32 bit count)
  HWREG(WTIMER5_BASE+TIMER_O_TBPR) =0;
  // set up timer A in capture mode (TAMR=3, TAAMS = 0), 
  // for edge time (TACMR = 1) and up-counting (TACDIR = 1)
  HWREG(WTIMER5_BASE+TIMER_O_TBMR) =(HWREG(WTIMER5_BASE+TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) 
    |(TIMER_TBMR_TBCDIR|TIMER_TBMR_TBCMR|TIMER_TBMR_TBMR_CAP);
  
  // To set the event to rising edge, we need to modify the TAEVENT bits 
  // in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
  HWREG(WTIMER5_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEVENT_M;
  
  // Now Set up the port to do the capture (clock was enabled earlier)
  // start by setting the alternate function for Port D bit 7 (WT5CCP1)
  HWREG(GPIO_PORTD_BASE+GPIO_O_AFSEL) |=BIT7HI;
  
  // Then, map bit 7's alternate function to WT0CCP0
  // 7 is the mux value to select WT0CCP0, 28 to shift it over to the
  // right nibble for bit 7 (4 bits/nibble * 7 bits)
  HWREG(GPIO_PORTD_BASE+GPIO_O_PCTL) =
    (HWREG(GPIO_PORTD_BASE+GPIO_O_PCTL) &0x0fffffff) + (7<<28);
  
  // Enable pin on Port D for digital I/O
  HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) |=BIT7HI;
  // make pin 7 on Port D into an input
  HWREG(GPIO_PORTD_BASE+GPIO_O_DIR) &=BIT7LO;
  
  // back to the timer to enable a local capture interrupt
  HWREG(WTIMER5_BASE+TIMER_O_IMR) |=TIMER_IMR_CBEIM;
  // enable the Timer A in Wide Timer 5 interrupt in the NVIC
  // it is interrupt number 105 so apppears in EN3 at bit 9
  HWREG(NVIC_EN3) |= BIT9HI;
  // make sure interrupts are enabled globally
  __enable_irq();
  
  // now kick the timer off by enabling it and enabling the timer to
  // stall while stopped by the debugger
  HWREG(WTIMER5_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN|TIMER_CTL_TBSTALL);


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
     PostBeaconDetect

 Parameters
     ES_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue

****************************************************************************/
bool PostBeaconDetect(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunBeaconDetect

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Search for and lock onto beacons.
****************************************************************************/
ES_Event_t RunBeaconDetect(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (ThisEvent.EventType) {
    case CORPORATION_SET: {
      // Store corporation
      OurCorporation = (Corporation_t) ThisEvent.EventParam;
      break;
    }
    
    case START_BEACON_DETECT: {
        if(getMinerAttached()){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = MINER_ATTACHED;
        PostRobotFSM(ThisEvent);
      }else{
        // Store desired periods
        CurrentState = Scanning;
        // Begin Rotating
        ES_Event_t DriveEvent;
        DriveEvent.EventType = RotateCW;
        PostDriveSM(DriveEvent);
        ES_Timer_InitTimer(BEACON_TIMER, DETECT_RATE);
        
        // Start timer for giving up looking in this location
        ES_Timer_InitTimer(BEACON_NOT_FOUND_TIMER, DETECTION_FAILED_TIME);
      }
      break;
    }
    
    case RESUME_BEACON_DETECT: {
      // Following a collision begin with wiggling
      // Only do so if we're looking for the beacon (Not in waiting state)
      if(CurrentState != Waiting){
        CurrentState = Wiggling;
        // Reset how much we need to wiggle
        WiggleDegree = WIGGLE_INCREMENT;
        ES_Event_t DriveEvent;
        DriveEvent.EventType = RotateCCWAngle;
        DriveEvent.EventParam = WiggleDegree;
        PostDriveSM(DriveEvent);
        // Start detection timer
        ES_Timer_InitTimer(BEACON_TIMER, DETECT_RATE);

        // Start timer for giving up looking for miner in this location
        ES_Timer_InitTimer(BEACON_NOT_FOUND_TIMER, DETECTION_FAILED_TIME);
      }else if(getMinerAttached()){
        ES_Event_t ThisEvent;
        ThisEvent.EventType = MINER_ATTACHED;
        PostRobotFSM(ThisEvent);
      }else{
        //If we're not starting with a wiggle, jump straight back into beacon detection
        ES_Event_t NewEvent;
        NewEvent.EventType = START_BEACON_DETECT;
        PostBeaconDetect(NewEvent);
      }
      break;
    }
    
    case ES_TIMEOUT: {
      if(ThisEvent.EventParam == BEACON_TIMER){
        if (CurrentState == Scanning || CurrentState == Wiggling) {
          // Check if desired period has been found
          CurrentMiner = IsPeriodGood();
          if (CurrentMiner != NoMiner) {
            // Stop spinning
            ES_Event_t DriveEvent;
            DriveEvent.EventType = Stop;
            PostDriveSM(DriveEvent);
            
            //Stop the beacon not found timer
            ES_Timer_StopTimer(BEACON_NOT_FOUND_TIMER);
            
            // Post event that beacon has been found
            ES_Event_t NewEvent;
            NewEvent.EventType = BEACON_FOUND;
            NewEvent.EventParam = CurrentMiner;
            PostRobotFSM(NewEvent);
                        
            CurrentState = InitialApproach;
            //Reinit WiggleCounter
            WiggleCounter = 0;
            
            //Initial Approach, rotate a bit CCW and move straight from there
            ES_Event_t AdjustAngleEvent;
            AdjustAngleEvent.EventType = RotateCCWAngle; 
            AdjustAngleEvent.EventParam = ANGLE_CORRECTION;
            PostDriveSM(AdjustAngleEvent);

            // Initialize locked on timer
            ES_Timer_InitTimer(BEACON_TIMER, CONFIRM_RATE);
            // Clear period so we can confirm we continue to measure
            Period = 0;
            } else {
            // Re-initialize detection timer
            ES_Timer_InitTimer(BEACON_TIMER, DETECT_RATE);
            }
          } else if (CurrentState == InitialApproach) {
            // Confirm that robot is still measuring period as we move towards it
            if (Period > 1.05*CurrentBeaconPeriod*TicksPerUS ||
              Period < 0.95*CurrentBeaconPeriod*TicksPerUS) {
                // Period being seen is no longer within the threshold
                // Post event that miner has been lost
                
                ES_Event_t NewEvent;
                NewEvent.EventType = BEACON_LOST;
                PostRobotFSM(NewEvent);
                
                // If we reinitialize the timer
                ES_Timer_InitTimer(BEACON_TIMER, CONFIRM_RATE);
                Period = 0;
                
            } else {
              // Re-initialize timer
              ES_Timer_InitTimer(BEACON_TIMER, CONFIRM_RATE);
              // Clear period so can check next time
              Period = 0;
            }
          }
    }else if(ThisEvent.EventParam==BEACON_NOT_FOUND_TIMER){ //If we spin for the timer length and dont see a beacon
            
      //Set the beacon not found flag in Drive SM
      SetBeaconNotFoundFlag(true);
      
      //Post an event to drive forward a specified distance
      ES_Event_t DriveEvent;
      DriveEvent.EventType = MoveFwdDist;
      DriveEvent.EventParam = NO_BEACON_MOVE_DISTANCE;
      PostDriveSM(DriveEvent);
    }
      break;
    }
    
    case EV_ROTATION_COMPLETED:{
      if(CurrentState == InitialApproach){
        // Adjusted our angle, now move forward some
        ES_Event_t FwdEvent;
        FwdEvent.EventType = MoveFwdDist;
        FwdEvent.EventParam = STRAIGHT_INCREMENT;
        PostDriveSM(FwdEvent);
        ES_Timer_InitTimer(BEACON_TIMER, CONFIRM_RATE);
      
      } else if (CurrentState == Wiggling){  
        if(WiggleCounter > WIGGLE_COUNT_MAX){
          ES_Event_t NewEvent;
          NewEvent.EventType = BEACON_LOST;
          PostRobotFSM(NewEvent);
          
          // Go back to scanning
          CurrentState = Scanning;
          ES_Event_t DriveEvent;
          DriveEvent.EventType = RotateCW;
          PostDriveSM(DriveEvent);
        
        } else {
          WiggleCounter++;
          // Increase scanning area
          uint16_t deg = abs(WiggleDegree);
          WiggleDegree = (deg + WIGGLE_INCREMENT)*-2;
          
          if(WiggleDegree < 0){
            // Rotate CW just by our chosen convention
            ES_Event_t RotateEvent;
            RotateEvent.EventType = RotateCWAngle;
            RotateEvent.EventParam = deg;
            PostDriveSM(RotateEvent);
          } else if (WiggleDegree > 0){
            ES_Event_t RotateEvent;
            RotateEvent.EventType = RotateCCWAngle;
            RotateEvent.EventParam = deg;
            PostDriveSM(RotateEvent);
          }
        // Start detection timer
        ES_Timer_InitTimer(BEACON_TIMER, DETECT_RATE);
        }
      }
      break;
    }
    case EV_STRAIGHT_COMPLETED:{
      if(CurrentState == InitialApproach){
        // Done moving straight, now wiggle since the miner hasn't been attached yet
        CurrentState = Wiggling;
        WiggleDegree = WIGGLE_INCREMENT;
        // Reset how much we need to wiggle
        ES_Event_t DriveEvent;
        DriveEvent.EventType = RotateCCWAngle;
        DriveEvent.EventParam = WiggleDegree;
        PostDriveSM(DriveEvent);
        // Start detection timer
        ES_Timer_InitTimer(BEACON_TIMER, DETECT_RATE);

        // Start timer for giving up looking for miner in this location
        ES_Timer_InitTimer(BEACON_NOT_FOUND_TIMER, DETECTION_FAILED_TIME);
      }
      break;
    }
    
    case END_BEACON_DETECT: {
      if (CurrentState == Scanning) {
        ES_Event_t DriveEvent;
        DriveEvent.EventType = Stop;
        PostDriveSM(DriveEvent);
      }
      CurrentState = Waiting;
      ES_Timer_StopTimer(BEACON_TIMER);
      break;
    }

    
    default:
      break;
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryBeaconDetect

 Parameters
     None

 Returns
     MinerName_t The current miner being tracked

 Description
     returns the current miner being tracked

****************************************************************************/
MinerName_t QueryCurrentMiner(void)
{
  return CurrentMiner;
}

/****************************************************************************
 Function
   BeaconResponse

 Parameters
   None

 Returns
   None

 Description
   ISR to save the period measured by beacon interrupt.
****************************************************************************/
void BeaconResponse(void)
{
  uint32_t ThisCapture;
  // start by clearing the source of the interrupt, the input capture event
  HWREG(WTIMER5_BASE+TIMER_O_ICR) = TIMER_ICR_CBECINT;
  
  // now grab the captured value and calculate the period
  ThisCapture=HWREG(WTIMER5_BASE+TIMER_O_TBR);
  Period=ThisCapture-LastCapture; 
  
  // update LastCapture to prepare for the next edge
  LastCapture=ThisCapture;
}
/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
   IsPeriodGood

 Parameters
   None

 Returns
   MinerName_t - enum of miner identified

 Description
   Checks if most recently measured period is within 5% of either
   desirable beacon period, returns that miner name if so. Stores period
   of matched miner beacon if detected.
   
****************************************************************************/
MinerName_t IsPeriodGood(void)
{
  if (OurCorporation == Corp1) {
    // If beacon matches within 5% of one the desired periods,
    // return miner and store period
    if (Period < 1.05*CKH1_PERIOD*TicksPerUS  &&
      Period > 0.95*CKH1_PERIOD*TicksPerUS)
    {
      if (!AreWeMining(MINER_1)){
        CurrentBeaconPeriod = CKH1_PERIOD;
        return CKH1;
      }
    }
    
    if (Period < 1.05*CKH2_PERIOD*TicksPerUS  &&
      Period > 0.95*CKH2_PERIOD*TicksPerUS)
    {
      if (!AreWeMining(MINER_2)){
        CurrentBeaconPeriod = CKH2_PERIOD;
        return CKH2;
      }
    }
    
  } else if (OurCorporation == Corp2) {
    // If beacon matches within 5% of one the desired periods,
    // return miner and store period
    if (Period < 1.05*GHI1_PERIOD*TicksPerUS  &&
      Period > 0.95*GHI1_PERIOD*TicksPerUS)
    {
      if (!AreWeMining(MINER_1)){
        CurrentBeaconPeriod = GHI1_PERIOD;
        return GHI1;
      }
    }
    
    if (Period < 1.05*GHI2_PERIOD*TicksPerUS  &&
      Period > 0.95*GHI2_PERIOD*TicksPerUS)
    {
      if (!AreWeMining(MINER_2)){
        CurrentBeaconPeriod = GHI2_PERIOD;
        return GHI2;
      }
    }
  }
  // Otherwise return no miner
  return NoMiner;
}
