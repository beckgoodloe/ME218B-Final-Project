/****************************************************************************

  Header file for state machine to manage finding
  beacons
 ****************************************************************************/

#ifndef BeaconDetect_H
#define BeaconDetect_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  Waiting, Scanning, LockedOn, InitialApproach, Wiggling
}BeaconState_t;

// typedefs for the miners
typedef enum
{
  CKH1, CKH2, GHI1, GHI2, NoMiner
}MinerName_t;

// Public Function Prototypes

bool InitBeaconDetect(uint8_t Priority);
bool PostBeaconDetect(ES_Event_t ThisEvent);
ES_Event_t RunBeaconDetect(ES_Event_t ThisEvent);
MinerName_t QueryCurrentMiner(void);
void BeaconResponse(void);

#endif /* BeaconDetect_H */

