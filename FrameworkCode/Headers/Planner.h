/****************************************************************************

  Header file for planner service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServPlanner_H
#define ServPlanner_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitPlanner(uint8_t Priority);
bool PostPlanner(ES_Event_t ThisEvent);
ES_Event_t RunPlanner(ES_Event_t ThisEvent);
void SetPlannerDest(uint8_t Location);
void SetPlannerStart(uint8_t Location);
uint8_t Distance(uint8_t, uint8_t);

#endif /* ServPlanner_H */

