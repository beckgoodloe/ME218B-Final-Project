/****************************************************************************

  Header file for the Drive SM
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef DriveSM_H
#define DriveSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPsuedoState, Stopped,
  DrivingFwd, DrivingRev, RotatingCW, RotatingCCW,
  RevAfterFrontCollision, FwdAfterBackCollision,
  CWAfterStall, CCWAfterStall,
  
  DrivingFwdDist, DrivingRevDist,
  RotatingCWAngle, RotatingCCWAngle
  
}DriveState_t;

// Public Function Prototypes

bool InitDriveSM(uint8_t Priority);
bool PostDriveSM(ES_Event_t ThisEvent);
ES_Event_t RunDriveSM(ES_Event_t ThisEvent);
DriveState_t QueryDriveSM(void);

void SetDriveDC(uint8_t dc);
void ImmediateStop(void);
void ResetCollisionCount(void);
void SetBeaconNotFoundFlag(bool);
void SetManyManyCollisions(bool);
#endif /* DriveSM_H */

