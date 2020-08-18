/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef StallService_H
#define StallService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitStallService(uint8_t Priority);
bool PostStallService(ES_Event_t ThisEvent);
ES_Event_t RunStallService(ES_Event_t ThisEvent);

#endif /* StallService_H */

