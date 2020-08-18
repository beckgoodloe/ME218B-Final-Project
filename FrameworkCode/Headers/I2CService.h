/****************************************************************************

  Header file for sample I2C implementation 
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef I2CService_H
#define I2CService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"    /* gets ES_Event_t type */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, Idle, Interpreting, Waiting4Busy, Waiting4Time
}I2CState_t;

// Public Function Prototypes

bool InitI2CService(uint8_t Priority);
bool PostI2CService(ES_Event_t ThisEvent);
ES_Event_t RunI2CService(ES_Event_t ThisEvent);
I2CState_t QueryI2CService(void);
bool IsI2C0Finished(void);
uint16_t I2C_GetClearValue(void);
uint16_t I2C_GetRedValue(void);
uint16_t I2C_GetGreenValue(void);
uint16_t I2C_GetBlueValue(void);

#endif /* I2CService_H */

/** From the README:

In your code, in the module(s) that you want to reference the getter functions,
#include the header I2CService.h

The getter functions available are:
uint16_t I2C_GetClearValue(void);
uint16_t I2C_GetRedValue(void);
uint16_t I2C_GetGreenValue(void);
uint16_t I2C_GetBlueValue(void);
*/
