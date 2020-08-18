/****************************************************************************
 Module
   CorporateManagement.c

 Revision
   1.0

 Description
   This is a file for managing the corporation selection under the
   Gen2 Events and Services Framework.

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "CorporateManagement.h"
#include "spudFSM.h"
#include "RobotFSM.h"
#include "BeaconDetect.h"

#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "bitdefs.h"
#include <stdint.h>
#include "inc/hw_memmap.h"

/*----------------------------- Module Defines ----------------------------*/
#define BLINK_TIME 500

/*---------------------------- Module Functions ---------------------------*/
static void Blink(void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

// Store our corporation
static Corporation_t OurCorporation;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitCorporateManagement

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and reads and posts the corporation.

****************************************************************************/
bool InitCorporateManagement(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // Initialize Port F for LEDs
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
  //Wait until clock is ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1)
  {
  }
  // Assign as digital pins
  HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= (BIT1HI | BIT2HI);
  // Set LED pins (PF3 and PF4) as outputs
  HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= (BIT1HI | BIT2HI);
  // Set LED pins low to start
  HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= (BIT1LO & BIT2LO);
  
  // Initialize Port E for corporation switch
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
  //Wait until clock is ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R4) != SYSCTL_PRGPIO_R4)
  {
  }
  // Set switch pin (PE5) as digital pin
  HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= BIT5HI;
  // Set switch pin (PE5) as input
  HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) &= BIT5LO;
  // Set switch pin as input pullup
  HWREG(GPIO_PORTE_BASE+GPIO_O_PUR) |= BIT5HI;
  

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
     PostCorporateManagement

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
bool PostCorporateManagement(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunCorporateManagement

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Responds to events to designate mining operations with lights
****************************************************************************/
ES_Event_t RunCorporateManagement(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  switch (ThisEvent.EventType) {
    case ES_INIT: {
      // Check our corporation
      uint8_t SwitchRead = HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) & BIT5HI;
      printf("Switch read: %u\r\n", SwitchRead);
      if (SwitchRead) {
        printf("corp 1\r\n");
        OurCorporation = Corp1;
      } else {
        printf("corp 2\r\n");
        OurCorporation = Corp2;
      }
      // Check our coporation and post our team to framework
      ES_Event_t CorpEvent;
      CorpEvent.EventType = CORPORATION_SET;
      CorpEvent.EventParam = OurCorporation;
      ES_PostAll(CorpEvent);
      PostSPUDFSM(CorpEvent);
      PostRobotFSM(CorpEvent);
      PostBeaconDetect(CorpEvent);
      
      // Set LED to designate team
      if (OurCorporation == Corp1) {
        // Light Red (CHK) LED
        HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT1HI;
      } else if (OurCorporation == Corp2) {
        // Light Blue (GHI) LED
        HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT2HI;
      }
      break;
    }
    
    case GAMESTATE_CHANGE: {
      if (ThisEvent.EventParam == PERMITS_ISSUED) {
        // Initialize blinking via timer
        ES_Timer_InitTimer(BLINK_TIMER, BLINK_TIME);
        Blink();
      } else if (ThisEvent.EventParam == PERMITS_EXPIRED) {
        // End blink routine, set LED to high
        ES_Timer_StopTimer(BLINK_TIMER);
        if (OurCorporation == Corp1) {
          // Light Red (CHK) LED
          HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT1HI;
        } else if (OurCorporation == Corp2) {
          // Light Blue (GHI) LED
          HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT2HI;
      }
        
      }
      break;
    }
    
    case ES_TIMEOUT: {
      if (ThisEvent.EventParam == BLINK_TIMER) {
        // Blink LED and reinitialize blink timer
        ES_Timer_InitTimer(BLINK_TIMER, BLINK_TIME);
        Blink();
      }
      break;
    }
    
    default:
      break;
  }
  
  return ReturnEvent;
  
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
   Blink

 Parameters
   None

 Returns
   None

 Description
   Toggle the current state of our corporation's LED
****************************************************************************/
void Blink(void) {
  // Store LED state
  static bool LEDon = true;
  // If LEDOn, set our corp's LED low
  if (LEDon) {
    if (OurCorporation == Corp1) {
      // Turn off Red (CHK) LED
      HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT1LO;
    } else if (OurCorporation == Corp2) {
      // Turn off Blue (GHI) LED
      HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT2LO;
    }
    LEDon = false;
  } else {
    // if LEDOff, set our corp's LED high
    if (OurCorporation == Corp1) {
      // Light Red (CHK) LED
      HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT1HI;
    } else if (OurCorporation == Corp2) {
      // Light Blue (GHI) LED
      HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT2HI;
    }
    LEDon = true;
  }
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

