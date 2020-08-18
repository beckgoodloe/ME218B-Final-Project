/****************************************************************************
 Module
   MotorControl.c

 Revision
   1.0

 Description
   This is a file for implementing basic control of 2 DC motors.

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "MotorControl.h"
#include "PWMModule.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "bitdefs.h"
#include <stdint.h>
#include "inc/hw_memmap.h"

/*----------------------------- Module Defines ----------------------------*/
#define MOTOR_A 0
#define MOTOR_B 1

#define MOTOR_A_HI BIT0HI
#define MOTOR_A_LO BIT0LO

#define MOTOR_B_HI BIT4HI
#define MOTOR_B_LO BIT4LO

/*---------------------------- Module Functions ---------------------------*/
static void SetForward(uint8_t motor);
static void SetReverse(uint8_t motor);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MotorADirection;
static uint8_t MotorBDirection;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitMotorControl

 Parameters
     None

 Returns
     None

 Description
     Initializes PWM and I/O pins used for motor control.

****************************************************************************/
void InitMotorControl(void)
{
  // Initialize PWM module
  InitPWMModule();
  
  //Now initialize port B for direction pins
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
  //Wait until clock is ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1)
  {
  }
  // Assign as digital pins
  HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (MOTOR_A_HI | MOTOR_B_HI);
  // Set pin as outputs
  HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (MOTOR_A_HI | MOTOR_B_HI);
  // Set pins low to start
  HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= (MOTOR_A_LO & MOTOR_B_LO);
  
  // Set both motors to off
  StopMotors();
}

/****************************************************************************
 Function
     StopMotors

 Parameters
     None

 Returns
     None

 Description
     Halts both DC motors

****************************************************************************/
void StopMotors(void)
{
  // Set duty cycle to zero
  SetForward(MOTOR_A);
  SetForward(MOTOR_B);
  SetMotorADC(0);
  SetMotorBDC(0);
}

/****************************************************************************
 Function
   Rotate

 Parameters
   uint8_t: direction to rotate (defined in header file)
   uint8_t: angle to rotate (45, 90, or CONTINUOUS)

 Returns
   None

 Description
   Sets motors to spin in opposite directions to cause bot to rotate
   specified direction for the specified degree amount.

****************************************************************************/
void Rotate(uint8_t direction)
{  
  // Set direction
  if (direction == CCW) {
    // Set motor A forward, B backward
    SetForward(MOTOR_A);
    SetReverse(MOTOR_B);
  } else if (direction == CW) {
    // Set motor A backward, B forward
    SetReverse(MOTOR_A);
    SetForward(MOTOR_B);
  }
  
}

/****************************************************************************
 Function
   DriveStraight

 Parameters
   uint8_t: direction to travel (defined in header file)
   uint8_t: speed (duty cycle)

 Returns
   None

 Description
   Sets motors to travel in same specified direction.

****************************************************************************/
void DriveStraight(uint8_t direction)
{
  // Set motor directions based on input
  if (direction == FWD) {
    SetForward(MOTOR_A);
    SetForward(MOTOR_B);
  } else if (direction == REV) {
    SetReverse(MOTOR_A);
    SetReverse(MOTOR_B);
  }
  
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
   SetForward

 Parameters
   uint8_t: motor identifier

 Returns
   None

 Description
   Sets motor to travel forward.

****************************************************************************/
void SetForward(uint8_t motor)
{
  // Set motor to forward direction
  if (motor == MOTOR_A) {
    MotorADirection = FWD;
    // Set one H bridge pin high
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= MOTOR_A_LO;
  } else if (motor == MOTOR_B) {
    MotorBDirection = FWD;
    // Set one H bridge pin low
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= MOTOR_B_HI;
  }
}

/****************************************************************************
 Function
   SetReverse

 Parameters
   uint8_t: motor identifier

 Returns
   None

 Description
   Sets motor to travel in reverse.

****************************************************************************/
void SetReverse(uint8_t motor)
{
  // Set motor to reverse direction
  if (motor == MOTOR_A) {
    MotorADirection = REV;
    // Set one H bridge pin low
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= MOTOR_A_HI;
  } else if (motor == MOTOR_B) {
    MotorBDirection = REV;
    // Set one H bridge pin high
    HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= MOTOR_B_LO;
  }
}

void SetMotorADC(uint8_t DC){
  if (MotorADirection == FWD) {
    // Invert for forward (based on wiring)
    PWM_SetDuty(DC, PWMA);
  } else {
    PWM_SetDuty(100 - DC, PWMA);
  }
}

void SetMotorBDC(uint8_t DC){
  if (MotorBDirection == FWD) {
    PWM_SetDuty(100 - DC, PWMB);
  } else {
    // Invert for reverse (based on wiring)
    PWM_SetDuty(DC, PWMB);
  }
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

