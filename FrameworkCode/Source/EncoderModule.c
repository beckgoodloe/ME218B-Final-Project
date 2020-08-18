/****************************************************************************
 Module
   EncoderModule.c

 Revision
   1.0.1

 Description
   Handles interactions with encoders on both left and right motors
   (Initializing interrupts for encoders, RPM calculation, and control)

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
#include "EncoderModule.h"
#include "MotorControl.h"
#include "DriveSM.h"
#include "ES_PostList.h"

#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "bitdefs.h"

/*----------------------------- Module Defines ----------------------------*/
// 60[s/min] * 40000000 [ticks/min] / 12 [encoder pulses per rev] / 50 [gear ratio] =  4000000
// Created to avoid integer overflow, divide by period [ticks] to get speed
#define RPM_FACTOR  4000000 
#define TICKS_PER_MS  40000

#define MAX_DUTY 100
#define MIN_DUTY 0

#define WHEEL_DIAMETER  76.2 // unit: mm (76.2 mm is 3 in for us imperially inclined)
#define PI  3.14159
#define MOTOR_TO_WHEEL_RATIO  145 // Empirically determined
#define TICKS_FOR_FULL_ROT  485 // Empirically determined
#define DEGREES_IN_CIRCLE 360

#define CONTROL_RATE 2 // unit: ms
#define TICK_THRESHOLD  3
#define SCALING_FACTOR  2
/*---------------------------- Module Functions ---------------------------*/
void InitEncoderCapture(void);
void LeftEncoderCaptureResponse(void);
void RightEncoderCaptureResponse(void);

void InitPeriodicInt(void);
void PeriodicControlResponse(void);

void ResetEncoderCount(void);
/*---------------------------- Module Variables ---------------------------*/
static uint8_t RightTargetDC;
static uint8_t LeftTargetDC;

static int32_t EncoderCountLeft;
static int32_t EncoderCountRight;

static uint32_t EncoderLimitLeft;
static uint32_t EncoderLimitRight;

static bool LeftLimitReached = false;
static bool RightLimitReached = false;

static int8_t Increment = 1;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitEncoderCapture

 Parameters

 Returns

 Description
    Initializes Wide Timer 0A and 0B, and pins PC4 and PC5 for encoder capture

****************************************************************************/
void InitEncoderCapture( void ){
  // start by enabling the clock to the timer (Wide Timer 0)
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
  
  // enable the clock to Port C
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
  // since we added this Port C clock init, we can immediately start
  // into configuring the timer, no need for further delay

  // make sure that timers (A and B) are disabled before configuring
  HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
  HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN;

  // set it up in 32bit wide (individual, not concatenated) mode
  // the constant name derives from the 16/32 bit timer, but this is a 32/64
  // bit timer so we are setting the 32bit mode
  HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
  
  // we want to use the full 32 bit count, so initialize the Interval Load
  // register to 0xffff.ffff (its default value :-)
  HWREG(WTIMER0_BASE+TIMER_O_TAILR) = 0xffffffff;
  HWREG(WTIMER0_BASE+TIMER_O_TBILR) = 0xffffffff;

  // we don't want any prescaler (it is unnecessary with a 32 bit count)
  HWREG(WTIMER0_BASE+TIMER_O_TAPR) = 0;
  HWREG(WTIMER0_BASE+TIMER_O_TBPR) = 0;

  // set up timer A in capture mode (TAMR=3, TAAMS = 0),
  // for edge time (TACMR = 1) and up-counting (TACDIR = 1)
  HWREG(WTIMER0_BASE+TIMER_O_TAMR) =
    (HWREG(WTIMER0_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) |
    (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
  
  // set up timer B in the exact same way
  HWREG(WTIMER0_BASE+TIMER_O_TBMR) =
    (HWREG(WTIMER0_BASE+TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) |
    (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP);
    
  // To set the event to rising edge, we need to modify the TAEVENT bits
  // in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
  HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;
  HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEVENT_M;
  
  // Now Set up the port to do the capture (clock was enabled earlier)
  // start by setting the alternate function for Port C bit 4 (WT0CCP0)
  // and Port C bit 5 (WT0CCP1)
  HWREG(GPIO_PORTC_BASE+GPIO_O_AFSEL) |= (BIT4HI|BIT5HI);
  
  // Then, map bit 4's alternate function to WT0CCP0
  // 7 is the mux value to select WT0CCP0, 16 to shift it over to the
  // right nibble for bit 4 (4 bits/nibble * 4 bits)
  HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) =
    (HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) & 0xfff0ffff) + (7<<16);
  // Map bit 5's alternate function to WT0CCP1,
  // 7 is the mux value, 20 to shift it over
  HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) =
    (HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) & 0xff0fffff) + (7<<20);
  
  // Enable pin on Port C for digital I/O
  HWREG(GPIO_PORTC_BASE+GPIO_O_DEN) |= (BIT4HI|BIT5HI);

  // make pin 4 and pin 5 on Port C into an input
  HWREG(GPIO_PORTC_BASE+GPIO_O_DIR) &= BIT4LO;
  HWREG(GPIO_PORTC_BASE+GPIO_O_DIR) &= BIT5LO;

  // back to the timer to enable a local capture interrupt
  HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_CAEIM;
  HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_CBEIM;

  // enable the Timer A in Wide Timer 0 interrupt in the NVIC
  // it is interrupt number 94 so apppears in EN2 at bit 30
  HWREG(NVIC_EN2) |= BIT30HI;
  // enable the Timer B in Wide Timer 0 interrupt in the NVIC
  // Interrupt number 95, EN2 (95>>5 = 2) and bit 31 (95%32)
  HWREG(NVIC_EN2) |= BIT31HI;

  // make sure interrupts are enabled globally
  __enable_irq();
  
  // now kick the timer off by enabling it and enabling the timer to
  // stall while stopped by the debugger
  HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
  HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
}

/****************************************************************************
InitPeriodicInt: 
  Initializes a periodic interrupt on Wide Timer 1A, used for control law
****************************************************************************/
void InitPeriodicInt( void ){
  // start by enabling the clock to the timer (Wide Timer 1)
  HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1; // kill a few cycles to let the clock get going
  while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R1) != SYSCTL_PRWTIMER_R1) {
  }
  // make sure that timer (Timer A) is disabled before configuring
  HWREG(WTIMER1_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
  // set it up in 32bit wide (individual, not concatenated) mode
  HWREG(WTIMER1_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
  // set up timer A in periodic mode so that it repeats the time-outs
  HWREG(WTIMER1_BASE+TIMER_O_TAMR) = (HWREG(WTIMER1_BASE+TIMER_O_TAMR)& ~TIMER_TAMR_TAMR_M)| TIMER_TAMR_TAMR_PERIOD;
  
  // set timeout based on control rate defined above
  HWREG(WTIMER1_BASE+TIMER_O_TAILR) = TICKS_PER_MS * CONTROL_RATE;
  // enable a local timeout interrupt
  HWREG(WTIMER1_BASE+TIMER_O_IMR) |= TIMER_IMR_TATOIM;

  // enable the Timer A in Wide Timer 1 interrupt in the NVIC 
  // it is interrupt number 96 so appears in EN3 (96>>5 = 3) at bit 0 (96%32 = 0)
  HWREG(NVIC_EN3) |= BIT0HI;

  // Change to a lower priority
  HWREG(NVIC_PRI24) = (HWREG(NVIC_PRI24) & ~NVIC_PRI24_INTA_M) + (1 << NVIC_PRI24_INTA_S); 

  // make sure interrupts are enabled globally
  __enable_irq();
  
  // now kick the timer off by enabling it and enabling the timer to // stall while stopped by the debugger
  HWREG(WTIMER1_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL); 
}

/****************************************************************************
LeftEncoderCapturerResponse: 
  Clears interrupt for left encoder and calculates the period based on captured value
****************************************************************************/
void LeftEncoderCaptureResponse( void ){
  // start by clearing the source of the interrupt, the input capture event
  HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;
  
  // Increase encoder count accordingly with wheel motion
  EncoderCountLeft = EncoderCountLeft + Increment;
  // If we're counting some distance
  if(EncoderLimitLeft > 0 && (EncoderCountLeft > EncoderLimitLeft)){
    LeftLimitReached = true;
    SetTargetLeftDC(0); // Stop Left Motor
    if(RightLimitReached){
      // Tell Drive SM that we're in the stopped state
      ES_Event_t ThisEvent;
      ThisEvent.EventType = EV_POSITION_REACHED;
      PostDriveSM(ThisEvent); 
      DisablePositionTracking();
    }
  }
}

/****************************************************************************
RightEncoderCapturerResponse: 
  Clears interrupt for left encoder and calculates the period based on captured value
****************************************************************************/
void RightEncoderCaptureResponse( void ){
  // start by clearing the source of the interrupt, the input capture event
   HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_CBECINT;
  
  // Increment Encoder count accordingly
  EncoderCountRight = EncoderCountRight + Increment;
  
  // If we're counting some distance
  if(EncoderLimitRight > 0 && (EncoderCountRight > EncoderLimitRight)){
    RightLimitReached = true;
    SetTargetRightDC(0); // Stop Right Motor
    if(LeftLimitReached){
      // Tell Drive SM that we're in the stopped state
      ES_Event_t ThisEvent;
      ThisEvent.EventType = EV_POSITION_REACHED;
      PostDriveSM(ThisEvent); 
      DisablePositionTracking();
    }
  }
}

/****************************************************************************
PeriodicControlResponse: 
  Works by setting duty cycle of each motor based on difference in encoder count
  The faster motor is set to be a lower duty cycle; these duty cycles are clamped
  to be between MAX_DUTY and MIN_DUTY defined above
****************************************************************************/
void PeriodicControlResponse( void ){
  // start by clearing the source of the interrupt
  HWREG(WTIMER1_BASE+TIMER_O_ICR) = TIMER_ICR_TATOCINT;
  int16_t DC_l = RightTargetDC;
  int16_t DC_r = LeftTargetDC;
  
  if(RightTargetDC == 0 || LeftTargetDC == 0){
    SetMotorADC(RightTargetDC);
    SetMotorBDC(LeftTargetDC);
    return;
  }
  
  int diff = EncoderCountRight - EncoderCountLeft;
  uint8_t DCOffset = abs(diff)* SCALING_FACTOR;
  if(diff > TICK_THRESHOLD)
  {
    // Right motor moved more so set DC on left to be higher
    DC_l = LeftTargetDC;
    DC_r = RightTargetDC - DCOffset;
  } else if (diff < -TICK_THRESHOLD){
    // Left motor moved more so set DC on right to be higher
    DC_l = LeftTargetDC - DCOffset;
    DC_r = RightTargetDC;
  }
  
  // Clamp between MIN and MAX duty cycles 
  if(DC_l > MAX_DUTY){   DC_l = MAX_DUTY;  } 
  else if (DC_l < MIN_DUTY) { DC_l = MIN_DUTY; }
  
  if(DC_r > MAX_DUTY){    DC_r = MAX_DUTY;  }
  else if (DC_r < MIN_DUTY)  {    DC_r = MIN_DUTY;  }
  
  SetMotorADC(DC_r);
  SetMotorBDC(DC_l);
}

/****************************************************************************
SetTargetRightDC: 
  Sets goal/nominal duty cycle to be used in the periodic control law for the
  right wheel
****************************************************************************/
void SetTargetRightDC(uint8_t NewTarget){
  RightTargetDC = NewTarget;
}

/****************************************************************************
SetTargetLeftDC: 
  Sets goal/nominal duty cycle to be used in the periodic control law for the
  left wheel
****************************************************************************/
void SetTargetLeftDC(uint8_t NewTarget){
  LeftTargetDC = NewTarget;
}

/****************************************************************************
SetEncoderLimitLeft: 
  Resets the left encoder count, sets the limit in the unit of ticks, and
  resets the limit reached boolean to false for good measure
****************************************************************************/
void SetEncoderLimitLeft(uint32_t limit){ //input unit [ticks]
  IncrementEncoderCount();
  EncoderCountLeft = 0;
  EncoderLimitLeft = limit;
  LeftLimitReached = false;
}

/****************************************************************************
SetEncoderLimitRight: 
  Resets the right encoder count, sets the limit in the unit of ticks, and
  resets the limit reached boolean to false for good measure
****************************************************************************/
void SetEncoderLimitRight(uint32_t limit){ //input unit [ticks]
  IncrementEncoderCount();
  EncoderCountRight = 0;
  EncoderLimitRight = limit;
  RightLimitReached = false;
}

/****************************************************************************
 Function
     SetEncoderDistanceMM

 Parameters
      Distance to travel in MM

 Returns
      Nothing

 Description
    Sets the encoder limit to move based on inputted distance in MM
    Travel [MM] * 1 rot/circumference of wheel [1/mm]
      * [motor to wheel rotations per revolution, determined empirically] 
          = Tick output sent to setter functions for the encoder limits
    
 ****************************************************************************/
void SetEncoderDistanceMM(uint16_t travel){
  float dist = travel*MOTOR_TO_WHEEL_RATIO/(WHEEL_DIAMETER*PI);
  SetEncoderLimitLeft(dist);
  SetEncoderLimitRight(dist);
}

/****************************************************************************
 Function
     SetEncoderDistanceDeg

 Parameters
      Amount to rotate in degrees

 Returns
      Nothing

 Description
    Sets the encoder limit to move based on inputted angle in degree
    The amount of ticks to rotate 360 degrees was determined empirically,
    The amount of degrees inputted is then multiplied by the amount of ticks it takes
    to rotate to get the number of ticks to be passed to the setter methods for
    the encoder limits
    
 ****************************************************************************/
void SetEncoderDistanceDeg(uint16_t degrees){
  float dist = degrees*TICKS_FOR_FULL_ROT/DEGREES_IN_CIRCLE;
  SetEncoderLimitLeft(dist);
  SetEncoderLimitRight(dist);
}

/****************************************************************************
ResetEncoderCount: 
  Resets both the left and right encoder count back to 0
****************************************************************************/
void ResetEncoderCount(void){
  EncoderCountLeft = 0;
  EncoderCountRight = 0;
}


/****************************************************************************
DisablePositionTracking:
  Set encoder limits to 0 so that they move indefinitely

****************************************************************************/
void DisablePositionTracking(void){
  // No need track limits
  EncoderLimitLeft = 0;
  EncoderLimitRight = 0;
}


/****************************************************************************
DecrementEncoderCount: 
  Sets the increment used in the encoder capture response to negative 1
  This is to be used when moving in the opposite direction of intended motion
  [i.e. a collision has been detected]
****************************************************************************/
void DecrementEncoderCount(void){
  Increment = -1;
}

/****************************************************************************
IncrementEncoderCount: 
  Sets the increment used in the encoder capture response to positive 1
****************************************************************************/
void IncrementEncoderCount(void){
  Increment = 1;
}

int32_t GetEncoderCountLeft(void){
  return EncoderCountLeft;
}

int32_t GetEncoderCountRight(void){
  return EncoderCountRight;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

