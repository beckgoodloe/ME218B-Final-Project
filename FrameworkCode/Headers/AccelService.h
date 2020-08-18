/****************************************************************************

  Header file for accelerometer service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServAccel_H
#define ServAccel_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitAccelService(uint8_t Priority);
bool PostAccelService(ES_Event_t ThisEvent);
ES_Event_t RunAccelService(ES_Event_t ThisEvent);
double QueryHeading(void);

uint16_t getMaxX(void);
uint16_t getMaxY(void);
void setAccelFlag(bool);
void detectCollision(void);
void setMultipleCollisionFlag(bool);
bool getMultipleCollisionFlag(void);

#endif /* ServAccel_H */

