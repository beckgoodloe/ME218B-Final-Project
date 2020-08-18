/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1

 Description
   This is the sample for writing event checkers along with the event
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.

 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// This gets us the prototype for ES_PostAll
#include "ES_Framework.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header &
// actual functionsdefinition
#include "EventCheckers.h"

#include "I2CService.h"
#include <stdint.h>
#include <stdbool.h>
#include "ADMulti.h"
#include "RobotFSM.h"
#include "AccelService.h"

#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "bitdefs.h"
#include <stdint.h>
#include "inc/hw_memmap.h"

// Empirically determined threshold for miner attached
#define MINER_DETECT_THRESHOLD 2700 // TODO: tune!

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so,
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if (IsNewKeyReady())   // new key waiting?
  {
    char Key = GetNewKey();
    ES_Event_t ThisEvent;
    switch(Key){
      case 's': {
        // Start the game
        ThisEvent.EventType = GAMESTATE_CHANGE;
        ThisEvent.EventParam = PERMITS_ISSUED;
        PostRobotFSM(ThisEvent);
        PostCorporateManagement(ThisEvent);
        break;
      }
      
      case 'p': {
        // Print debug
        ThisEvent.EventType = PRINT_DEBUG;
        ES_PostAll(ThisEvent);
        break;
      }
      
      case 'b': {
        // Beacon found
        // Note: will not change the state of BeaconDetect, which may be scanning
        ThisEvent.EventType = BEACON_FOUND;
        PostRobotFSM(ThisEvent);
        break;
      }     
      
      case 'a':{
        //printf("\nStopped\r\n");
        ImmediateStop();
        break;
      }
      case 'f':{
        ThisEvent.EventType = MoveFwd;
        //printf("\nDrive Forward\r\n");
        PostAccelService(ThisEvent);
        PostDriveSM(ThisEvent);
        break;
      }
      case 'r':{
        ThisEvent.EventType = MoveRev;
        //printf("\nDrive Reverse\r\n");
        PostAccelService(ThisEvent);
        PostDriveSM(ThisEvent);
        break;
      }
      case 'q':{
        printf("\nX Max = %u, Y Max = %u\r\n",getMaxX(),getMaxY());
        break;
      }
      
      case 'h': {
        printf("Current heading: %f\r\n", QueryHeading());
        break;
      }
      
      default:
        return false;
    }
    return true;
  }
  return false;
}

/****************************************************************************
 Function
   Check4Miner
 Parameters
   None
 Returns
   bool: true if a miner was detected or detached & posted
 Description
   checks to see if the state of the miner attachment has changed, and if so
   posts it to Robot FSM
****************************************************************************/
static bool MinerAttached = false;
bool Check4Miner(void)
{
  static uint32_t ADResults[4];
  ADC_MultiRead(ADResults);
  uint32_t ThisRead = ADResults[0];
  bool ReturnVal = false;
  
  if (MinerAttached) {
    // If miner was attached, see if sensor has dipped below threshold
    // (with 10% hysteresis)
    if (ThisRead <= 0.9*MINER_DETECT_THRESHOLD) {
      // Post that miner has detached, store state
      ES_Event_t ThisEvent;
      ThisEvent.EventType = MINER_DETACHED;
      PostRobotFSM(ThisEvent);
      MinerAttached = false;
      ReturnVal = true;
    }
  } else {
    // If miner was not attached, see if sensor has risen above threshold
    // (with 10% hysteresis)
    if (ThisRead >= 1.1*MINER_DETECT_THRESHOLD) {
      // Post that miner has attached, store state
      ES_Event_t ThisEvent;
      ThisEvent.EventType = MINER_ATTACHED;
      PostRobotFSM(ThisEvent);
      MinerAttached = true;
      ReturnVal = true;
    }
  }
  
  // If none detected, return false
  return ReturnVal;

}

/****************************************************************************
 Function
   getMinerAttached
 Parameters
   None
 Returns
   bool: true if a miner was detected or detached & posted
 Description
   queries whether a miner is attached currently
****************************************************************************/
bool getMinerAttached(void){
  return MinerAttached;
}
