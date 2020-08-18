/****************************************************************************

  Header file for SPUD communications Flat State Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef FSMSpud_H
#define FSMSpud_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// Define SPUD response
#define WAITING_FOR_PERMITS 0
#define PERMITS_ISSUED 1
#define SUDDEN_DEATH 2
#define PERMITS_EXPIRED 3

// Struct to keep track of a single miner
struct Miner_t
{
  uint8_t Location, WFlag, BFlag, UFlag, LocGood;
  bool CurrentlyMining;
};

// Struct to keep track of game state
struct Game_t
{
  uint8_t GameState;
  struct Miner_t Miner1, Miner2;
  uint8_t ExclLocation, SharedLocation1, SharedLocation2;
};

// Public Function Prototypes

bool InitSPUDFSM(uint8_t Priority);
bool PostSPUDFSM(ES_Event_t ThisEvent);
ES_Event_t RunSPUDFSM(ES_Event_t ThisEvent);
uint8_t QueryExclLocation(void);
uint8_t QueryCurrentLocation(void);
bool AreWeMining(uint8_t number) ;
uint8_t WhereShouldWeGo(void);

#endif /* FSMSpud_H */

