/****************************************************************************
 Module
   PWM Module

 Revision
   1.0

 Description
   This is a file that implements PWM functionality using Tiva hardware

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "PWMModule.h"

// the headers to access the GPIO subsystem
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_pwm.h"

// the headers to access the TivaWare Library
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "BITDEFS.H"

/*----------------------------- Module Defines ----------------------------*/
// 40,000 ticks per mS assumes a 40Mhz clock, we will use SysClk/32 for PWM
#define PWM_TICKS_PER_MS 40000/32

// set 6250 Hz frequency so .16mS period
#define PWM_FREQ  6250
#define ONE_SEC_IN_MS 1000

// program generator A to go to 1 at rising comare A, 0 on falling compare A
#define GenA_Normal (PWM_0_GENA_ACTCMPAU_ONE | PWM_0_GENA_ACTCMPAD_ZERO )
// program generator B to go to 1 at rising comare B, 0 on falling compare B  
#define GenB_Normal (PWM_0_GENB_ACTCMPBU_ONE | PWM_0_GENB_ACTCMPBD_ZERO )


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
void Set0DC(uint8_t Pin);
void Set100DC(uint8_t Pin);
void RestoreDC(uint8_t Pin);

void PWM_SetDuty(uint32_t DutyCycle, uint8_t Pin);
void PWM_SetPeriod(uint32_t NewPeriod);
/*---------------------------- Module Variables ---------------------------*/

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitPWMModule

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes two PWM modules.

****************************************************************************/
bool InitPWMModule(void)
{
  // start by enabling the clock to the PWM Module (PWM0)
  HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
  // enable the clock to Port B
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
  
  // Select the PWM clock as System Clock/32
  HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) |
    (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
  
  // make sure that the PWM module clock has gotten going
  while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0){
  }
  
  // disable the PWM while initializing
  HWREG( PWM0_BASE+PWM_O_0_CTL ) = 0;
  
  // program generators to go to 1 at rising compare A/B, 0 on falling compare A/B
  HWREG( PWM0_BASE+PWM_O_0_GENA) = GenA_Normal;
  HWREG(PWM0_BASE+PWM_O_0_GENB) = GenB_Normal;
  
  // Set the PWM period. Since we are counting both up & down, we initialize
  // the load register to 1/2 the desired total period. We will also program
  // the match compare registers to 1/2 the desired high time
  // Set a period of 6250 Hz --> .16 ms
  HWREG( PWM0_BASE+PWM_O_0_LOAD) = ((ONE_SEC_IN_MS*PWM_TICKS_PER_MS/PWM_FREQ))>>1;


  // Set the initial Duty cycle on A and B to 50% by programming the compare value
  // to 1/2 the period to count up (or down). Technically, the value to program
  // should be Period/2 - DesiredHighTime/2, but since the desired high time is 1/2
  // the period, we can skip the subtract
  HWREG( PWM0_BASE+PWM_O_0_CMPA) = HWREG( PWM0_BASE+PWM_O_0_LOAD)>>1;
  HWREG( PWM0_BASE+PWM_O_0_CMPB) = HWREG( PWM0_BASE+PWM_O_0_LOAD)>>1;
  
  // enable the PWM outputs
  HWREG( PWM0_BASE+PWM_O_ENABLE) |= (PWM_ENABLE_PWM1EN|PWM_ENABLE_PWM0EN);
  
  // now configure the Port B pins to be PWM outputs
  // start by selecting the alternate function for PB6
  HWREG(GPIO_PORTB_BASE+GPIO_O_AFSEL) |= (BIT7HI|BIT6HI);
  
  // now choose to map PWM to those pins, this is a mux value of 4 that we
  // want to use for specifying the function on bits 6 & 7
  HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) =
    (HWREG(GPIO_PORTB_BASE+GPIO_O_PCTL) &0x00ffffff) + (4<<(7*BITS_PER_NYBBLE)) +(4<<(6*BITS_PER_NYBBLE));  
  
  // Enable PB6,7 for digital I/O
  HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (BIT7HI|BIT6HI);
  // make PB6,7 into outputs
  HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (BIT7HI|BIT6HI);
  
  //Set the up down count mode, enable the PWM generator and make both generator updates locally synchronized to zero count
  HWREG(PWM0_BASE+PWM_O_0_CTL) = (PWM_0_CTL_MODE | PWM_0_CTL_ENABLE | PWM_0_CTL_GENAUPD_LS | PWM_0_CTL_GENBUPD_LS);

  return true;
}


/****************************************************************************
 Function
     SetDuty

 Parameters
     DutyCycle: desired duty cycle (int)
     Pin: desired PWM pin (PWMA or PWMB)

 Description
     Sets PWM module to desired duty cycle

****************************************************************************/
void PWM_SetDuty(uint32_t DutyCycle, uint8_t Pin){
  // Set pin A
  if(Pin == PWMA){
    if(DutyCycle >= 100){
      // Set 100 if at or higher than 100
      Set100DC(Pin);
    } else if (DutyCycle <= 0){
      // Set 0 if at or lower than 0
      Set0DC(Pin);
    } else {
      // Otherwise, restore the operation and set the compare time
      RestoreDC(Pin);
      HWREG(PWM0_BASE+PWM_O_0_CMPA) = (HWREG(PWM0_BASE+PWM_O_0_LOAD) * (100-DutyCycle)/100);
    }
  // Set pin B  
  } else if (Pin == PWMB){
    if(DutyCycle >= 100){
      // Set 100 if at or higher than 100
      Set100DC(Pin);
    } else if (DutyCycle <= 0){
      // Set 0 if at or lower than 0
      Set0DC(Pin);
    } else {
      // Otherwise, restore the operation and set the compare time
      RestoreDC(Pin);
      HWREG(PWM0_BASE+PWM_O_0_CMPB) = (HWREG(PWM0_BASE+PWM_O_0_LOAD) * (100-DutyCycle)/100);
    }
  }
}


/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
     Set0DC

 Parameters:
     Pin: desired PWM pin (PWMA or PWMB)

 Description
     Sets PWM module to 0 DC (full low output)

****************************************************************************/
void Set0DC(uint8_t Pin){
  // To program 0% DC, simply set the action on Zero to set the output to zero
  if(Pin == PWMA){
      HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ZERO;
  }
  else if (Pin == PWMB){
      HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ZERO;
  }
}

/****************************************************************************
 Function
     Set100DC

 Parameters
     Pin: desired PWM pin (PWMA or PWMB)

 Description
     Sets PWM module to 100 DC (full high output)
****************************************************************************/
void Set100DC(uint8_t Pin){
  // To program 100% DC, simply set the action on Zero to set the output to one
  if(Pin == PWMA){
    HWREG( PWM0_BASE+PWM_O_0_GENA) = PWM_0_GENA_ACTZERO_ONE;
  }
  else if (Pin == PWMB){
    HWREG( PWM0_BASE+PWM_O_0_GENB) = PWM_0_GENB_ACTZERO_ONE;
  }
}

/****************************************************************************
 Function
     RestoreDC

 Parameters
     Pin: desired PWM pin (PWMA or PWMB)

 Description
     Sets PWM module to previous duty cycle

****************************************************************************/
void RestoreDC(uint8_t Pin){
  // To restore the previos DC, simply set the action back to the normal actions
  if(Pin == PWMA){
    HWREG( PWM0_BASE+PWM_O_0_GENA) = GenA_Normal;
  }
  else if (Pin == PWMB){
    HWREG(PWM0_BASE+PWM_O_0_GENB) = GenB_Normal;
  }
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

