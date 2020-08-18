/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef RobotFSM_H
#define RobotFSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  WaitingToStart, SearchingForMiner, ApproachingMiner, MovingMiner
}RobotState_t;

// Public Function Prototypes

bool InitRobotFSM(uint8_t Priority);
bool PostRobotFSM(ES_Event_t ThisEvent);
ES_Event_t RunRobotFSM(ES_Event_t ThisEvent);
RobotState_t QueryRobotSM(void);
bool getMinerDeposited(void);
void setMinerDeposited(bool);

#endif /* RobotFSM_H */

