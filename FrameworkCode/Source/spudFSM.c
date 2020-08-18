/****************************************************************************
 Module
   spudFSM.c

 Revision
   1.0

 Description
   This is a file for implementing control of SPUD communications

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "spudFSM.h"
#include "serial.h"
#include "RobotFSM.h"
#include "CorporateManagement.h"
#include "BeaconDetect.h"
#include "DriveSM.h"
#include "planner.h"

/*----------------------------- Module Defines ----------------------------*/
// Check game state every 300ms
#define QUERY_RATE 25

// Define SPUD queries
#define QUERY_GAMESTATE 0xC0
#define QUERY_C1MLOC1 0xC1
#define QUERY_C1MLOC2 0xC2
#define QUERY_C2MLOC1 0xC3
#define QUERY_C2MLOC2 0xC4
#define QUERY_C1RESL 0xC6
#define QUERY_C2RESL 0xC8
#define QUERY_PUR1 0xC9
#define QUERY_PUR2 0xCA
#define EMPTY_QUERY 0x00

// Bit masks
#define LOC1_MASK 0x0f
#define GAMESTATE_MASK 0x03
#define UFLAG_MASK 0x10
#define BFLAG_MASK 0x20
#define WFLAG_MASK 0x40
#define LOCGOOD_MASK 0x80

// Maximum travel distance (units are relative)
#define MAX_DISTANCE 0xFF
/*---------------------------- Module Functions ---------------------------*/
static void ProcessByte(uint8_t byte);
static void PrintState(void);
static void CheckMiningStatus(void);
static bool IsOccupied(uint8_t Location);

/*---------------------------- Module Variables ---------------------------*/
// Store our most recent query as we await response
static uint8_t LastQuery;
// Keep track of where in our query array we are
static uint8_t CurrentIndex;
// Array of queries to issue periodically
static uint8_t QueryArray[] = {QUERY_GAMESTATE, QUERY_C1MLOC1, QUERY_C1MLOC2,
                               QUERY_C1RESL, QUERY_PUR1, QUERY_PUR2};
// Struct to track full state of game
static struct Game_t ThisGame;
// Bit to keep track of which corporation we are
static Corporation_t OurCorporation;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSPUDFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and
		 initializes serial communication library.

****************************************************************************/
bool InitSPUDFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // initialize index
  CurrentIndex = 0;
  // Initialize mining variables
  ThisGame.Miner1.CurrentlyMining = false;
  ThisGame.Miner2.CurrentlyMining = false;
  
	// Initialize serial library
	InitSerial();
  // Initialize corporation arbitrarily
  OurCorporation = Corp1;
	// Initialize timer to periodically query game
	ES_Timer_InitTimer(SPUD_TIMER, QUERY_RATE);
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
     PostSPUDFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue

****************************************************************************/
bool PostSPUDFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunSPUDFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Periodical query game state on ES_TIMEOUT, post any changes.
 
****************************************************************************/
ES_Event_t RunSPUDFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (ThisEvent.EventType) {
    case CORPORATION_SET: {
      // Set our corporation
      OurCorporation = (Corporation_t) ThisEvent.EventParam;
      if (OurCorporation == Corp2) {
        // Change miner location commands from default
        QueryArray[1] = QUERY_C2MLOC1;
        QueryArray[2] = QUERY_C2MLOC2;
        QueryArray[3] = QUERY_C2RESL;
      }
      break;
    }
    
    case ES_TIMEOUT: {
      if (ThisEvent.EventParam == SPUD_TIMER) {
        // Query releavant command
        uint8_t CurrentQuery = QueryArray[CurrentIndex];
        // Send query with two blank transmissions
        TransmitSerial(CurrentQuery);
        TransmitSerial(EMPTY_QUERY);
        TransmitSerial(EMPTY_QUERY);
        // Store most recent query and increment index
        LastQuery = CurrentQuery;
        CurrentIndex++;
        // Make sure index does not exceed bounds of array
        CurrentIndex = CurrentIndex % 6;
        // Re-initialize timer
        ES_Timer_InitTimer(SPUD_TIMER, QUERY_RATE); 
      }
      break;
    }
    
    case BYTE_RECEIVED: {
      // Parse reeived data
      ProcessByte(ThisEvent.EventParam);
      break;
    }
    
    case PRINT_DEBUG: {
      // Print our understanding of game state, for debugging
      PrintState();
    }
    
    default:
      break;
  }

  return ReturnEvent;
}

/****************************************************************************
 Function
    QueryExclLocation

 Parameters
    None

 Returns
    uint8_t: Location of current exclusive permits

 Description
    Returns location of exclusive permits

****************************************************************************/
uint8_t QueryExclLocation(void) {
  return ThisGame.ExclLocation;
}

/****************************************************************************
 Function
    WhereShouldWeGo

 Parameters
    None

 Returns
    uint8_t: Location of where to deposit miner

 Description
    Compares exclusive and neutral mining regions. Returns the closest one
    that we're not already on.

****************************************************************************/
uint8_t WhereShouldWeGo(void) {
  uint8_t OurLocation = QueryCurrentLocation();
  
  // Calculate distance to our exclusive location
  uint8_t ExclDist = Distance(OurLocation, ThisGame.ExclLocation);
  // If we already have a miner there, max its distance so we don't go for it
  if (IsOccupied(ThisGame.ExclLocation)) {
    ExclDist = MAX_DISTANCE;
  }
  
  // Calculate distance to neutral zone 1
  uint8_t Neut1Dist = Distance(OurLocation, ThisGame.SharedLocation1);
  // If we already have a miner there, max its distance so we don't go for it
  if (IsOccupied(ThisGame.SharedLocation1)) {
    Neut1Dist = MAX_DISTANCE;
  }
  
  // Calculate distance to neutral zone 2
  uint8_t Neut2Dist = Distance(OurLocation, ThisGame.SharedLocation2);
  // If we already have a miner there, max its distance so we don't go for it
  if (IsOccupied(ThisGame.SharedLocation2)) {
    Neut2Dist = MAX_DISTANCE;
  }
  
  // Return location with minimum distance
  if (ExclDist <= Neut1Dist && ExclDist <= Neut2Dist) {
    return ThisGame.ExclLocation;
  } else if (Neut1Dist <= ExclDist && Neut1Dist <= Neut2Dist) {
    return ThisGame.SharedLocation1;
  } else {
    return ThisGame.SharedLocation2;
  }
}

/****************************************************************************
 Function
    QueryCurrentLocation

 Parameters
    None

 Returns
    uint8_t: Location of current miner

 Description
    Returns location of current miner

****************************************************************************/
uint8_t QueryCurrentLocation(void) {
  // Query current miner
  MinerName_t CurrentMiner = QueryCurrentMiner();
  // Return the location of that miner
  if (CurrentMiner == CKH1 || CurrentMiner == GHI1) {
    return ThisGame.Miner1.Location;
  } else { // Even if no miner, return a location
    return ThisGame.Miner2.Location; // NOTE: will return location even if not good
  }
}

/****************************************************************************
 Function
    AreWeMining

 Parameters
    uint8_t: Miner number, 1 or 2

 Returns
    bool: whether we are mining the specified miner

 Description
    Returns whether we are currently mining the specified miner

****************************************************************************/
bool AreWeMining(uint8_t number) {
  // Return the mining status of specified miner
  if (number == 1) {
    return ThisGame.Miner1.CurrentlyMining;
  } else if (number == 2) {
    return ThisGame.Miner2.CurrentlyMining;
  }
  return false;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/****************************************************************************
 Function
     ProcessByte

 Parameters
    uint8_t: byte received over serial

 Returns
		None

 Description
    Parses a byte received from the SPUD and stores it.

****************************************************************************/
void ProcessByte(uint8_t byte)
{
  // Handle it based on what the most recent query was
  switch (LastQuery) {
    
    // If last query was gamestate, check if it has changed
    case QUERY_GAMESTATE: {
      // Mask out game state
      uint8_t NewState = byte & GAMESTATE_MASK;
      // Post event only if state has changed
      if (ThisGame.GameState != NewState) {
        // Store new state
        ThisGame.GameState = NewState;
        // Post game state change event
        ES_Event_t ThisEvent;
        ThisEvent.EventType = GAMESTATE_CHANGE;
        ThisEvent.EventParam = NewState;
        ImmediateStop();
        PostRobotFSM(ThisEvent);
        PostCorporateManagement(ThisEvent);
      }
      break;
    }
    
    // Regardless of team, store location info of miner 1
    case QUERY_C1MLOC1:
    case QUERY_C2MLOC1: {
      // Store flags
      ThisGame.Miner1.UFlag = (byte & UFLAG_MASK) >> 4;
      ThisGame.Miner1.BFlag = (byte & BFLAG_MASK) >> 5;
      ThisGame.Miner1.WFlag = (byte & WFLAG_MASK) >> 6;
      ThisGame.Miner1.LocGood = (byte & LOCGOOD_MASK) >> 7;
      // If location is good and new, store location
      if (ThisGame.Miner1.LocGood) {
        uint8_t NewLocation = byte & LOC1_MASK;
        if (NewLocation != ThisGame.Miner1.Location) {
          ThisGame.Miner1.Location = NewLocation;
          // If we are carrying this miner, post its location change
          MinerName_t CurrentMiner = QueryCurrentMiner();
          if (CurrentMiner == CKH1 || CurrentMiner == GHI1) {
            ES_Event_t NewEvent;
            NewEvent.EventType = MINER_LOCATION;
            NewEvent.EventParam = NewLocation;
            PostRobotFSM(NewEvent);
          }            
        } 
      } else {
          //printf("MINER 1 is bad! Bleh.\r\n");
       }
      // Check whether mining status has changed
      CheckMiningStatus();
      break;
    }
    
    // Regardless of team, store location info of miner 2
    case QUERY_C1MLOC2:
    case QUERY_C2MLOC2: {
      ThisGame.Miner2.UFlag = (byte & UFLAG_MASK) >> 4;
      ThisGame.Miner2.BFlag = (byte & BFLAG_MASK) >> 5;
      ThisGame.Miner2.WFlag = (byte & WFLAG_MASK) >> 6;
      ThisGame.Miner2.LocGood = (byte & LOCGOOD_MASK) >> 7;
      // If location is good and new, store location
      if (ThisGame.Miner2.LocGood) {
        uint8_t NewLocation = byte & LOC1_MASK;
        if (NewLocation != ThisGame.Miner2.Location) {
          ThisGame.Miner2.Location = NewLocation;
          // If we are carrying this miner, post its location change
          MinerName_t CurrentMiner = QueryCurrentMiner();
          if (CurrentMiner == CKH2 || CurrentMiner == GHI2) {
            ES_Event_t NewEvent;
            NewEvent.EventType = MINER_LOCATION;
            NewEvent.EventParam = NewLocation;
            PostRobotFSM(NewEvent);
          }            
        }
      } else {
          //printf("MINER 2 is bad! Bleh.\r\n");
      }
      // Check whether mining status has changed
      CheckMiningStatus();
      break;
    }
    
    case QUERY_PUR1: {
      // Store location exclusive to our corporation
      if (OurCorporation == Corp1) {
        ThisGame.ExclLocation = byte & LOC1_MASK;
      } else {
        ThisGame.ExclLocation = byte >> 4;
      }
      break;
    }
    case QUERY_PUR2: {
      // Store both shared locations
      ThisGame.SharedLocation1 = byte & LOC1_MASK;
      ThisGame.SharedLocation2 = byte >> 4;
      break;
    }
    
    default:
      break;
  }
  return;
}

/****************************************************************************
 Function
    CheckMiningStatus

 Parameters
    None

 Returns
		None

 Description
    Checks whether currently mining, updates mining state variables, and
    posts whether mining.

****************************************************************************/
void CheckMiningStatus() {
  // Based on most recent info, are we actually mining with miner 1?
  bool ActuallyMining1 = (ThisGame.Miner1.Location == ThisGame.ExclLocation
    || ThisGame.Miner1.Location == ThisGame.SharedLocation1
    || ThisGame.Miner1.Location == ThisGame.SharedLocation2
  );
  // Only know we are mining if location is good
  ActuallyMining1 &= ThisGame.Miner1.LocGood;
  
  // Based on most recent info, are we actually mining with miner 2?
  bool ActuallyMining2 = (ThisGame.Miner2.Location == ThisGame.ExclLocation
    || ThisGame.Miner2.Location == ThisGame.SharedLocation1
    || ThisGame.Miner2.Location == ThisGame.SharedLocation2
  );
  // Only know we are mining if location is good
  ActuallyMining2 &= ThisGame.Miner2.LocGood;
  
  // If both are in the same location, we are definitely not mining
  if (ThisGame.Miner1.Location == ThisGame.Miner2.Location
    && ThisGame.Miner1.LocGood && ThisGame.Miner2.LocGood) {
      ActuallyMining1 = false;
      ActuallyMining2 = false;
  }
  
  // Update our states and post if changed
  if (ThisGame.Miner1.CurrentlyMining) {
    // Check if we've stopped mining
    if (!ActuallyMining1) {
      // Post this to alert we've stopped mining
      ES_Event_t NewEvent;
      NewEvent.EventType = STOPPED_MINING;
      NewEvent.EventParam = 1;
      ThisGame.Miner1.CurrentlyMining = ActuallyMining1;
    }
  } else {
    // Check if we've started mining
    if (ActuallyMining1) {
      ThisGame.Miner1.CurrentlyMining = ActuallyMining1;
    }
  }
  
  // Update our states and post if changed
  if (ThisGame.Miner2.CurrentlyMining) {
    // Check if we've stopped mining
    if (!ActuallyMining2) {
      // Post this
      ES_Event_t NewEvent;
      NewEvent.EventType = STOPPED_MINING;
      NewEvent.EventParam = 2;
      ThisGame.Miner2.CurrentlyMining = ActuallyMining2;
    }
  } else {
    // Check if we've started mining
    if (ActuallyMining2) {
      ThisGame.Miner2.CurrentlyMining = ActuallyMining2;
    }
  }
}
  
/****************************************************************************
 Function
     PrintState

 Parameters
    None

 Returns
		None

 Description
    Prints current understanding of game

****************************************************************************/
void PrintState(void) {
  // Print game struct for debugging
  printf("-------------------------------\r\n");
  printf("Game state: %u\r\n", ThisGame.GameState);
  printf("Miner 1:\r\n");
  printf("  Location: %u\r\n", ThisGame.Miner1.Location);
  printf("  Uflag: %u\r\n", ThisGame.Miner1.UFlag);
  printf("  Bflag: %u\r\n", ThisGame.Miner1.BFlag);
  printf("  Wflag: %u\r\n", ThisGame.Miner1.WFlag);
  printf("  LocGood: %u\r\n", ThisGame.Miner1.LocGood);
  printf("  Currently Mining: %u\r\n", ThisGame.Miner1.CurrentlyMining);
  printf("Miner 2:\r\n");
  printf("  Location: %u\r\n", ThisGame.Miner2.Location);
  printf("  Uflag: %u\r\n", ThisGame.Miner2.UFlag);
  printf("  Bflag: %u\r\n", ThisGame.Miner2.BFlag);
  printf("  Wflag: %u\r\n", ThisGame.Miner2.WFlag);
  printf("  LocGood: %u\r\n", ThisGame.Miner2.LocGood);
  printf("  Currently Mining: %u\r\n", ThisGame.Miner2.CurrentlyMining);
  printf("Exclusive location: %u\r\n", ThisGame.ExclLocation);
  printf("Shared location 1: %u\r\n", ThisGame.SharedLocation1);
  printf("Shared location 2: %u\r\n", ThisGame.SharedLocation2);
  printf("-------------------------------\r\n");
}

/****************************************************************************
 Function
    IsOccupied

 Parameters
    uint8_t: location under question

 Returns
    bool: whether location under question is already occupied by one
          of our miners

 Description
    Returns whether location under question is already occupied by one
    of our miners

****************************************************************************/
bool IsOccupied(uint8_t Location) {
  if (Location == ThisGame.Miner1.Location && ThisGame.Miner1.LocGood) {
    return true;
  }
  
  if (Location == ThisGame.Miner2.Location && ThisGame.Miner2.LocGood) {
    return true;
  }
  
  return false;
}
