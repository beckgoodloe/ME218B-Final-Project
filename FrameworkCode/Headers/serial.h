/****************************************************************************

  Header file for SPI module

 ****************************************************************************/

#ifndef Serial_H
#define Serial_H

#include "ES_Types.h"

// Public Function Prototypes

void InitSerial(void);
void TransmitSerial(uint8_t message);
void ReceiveSerial(void);

#endif /* Serial_H */

