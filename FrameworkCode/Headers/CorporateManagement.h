/****************************************************************************

  Header file for corporation detection service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServCorp_H
#define ServCorp_H

#include "ES_Types.h"

// Enumerate the corporations
typedef enum
{
  Corp1, Corp2
}Corporation_t;

// Public Function Prototypes

bool InitCorporateManagement(uint8_t Priority);
bool PostCorporateManagement(ES_Event_t ThisEvent);
ES_Event_t RunCorporateManagement(ES_Event_t ThisEvent);
Corporation_t ReadCorporation(void);

#endif /* ServCorp_H */
