/****************************************************************************

  Header file for Accelerometer SPI module

 ****************************************************************************/

#ifndef AccelerometerSerial_H
#define AccelerometerSerial_H

#include "ES_Types.h"

// Public Function Prototypes

void InitAccelSerial(void);
void TransmitAccelSerial(uint8_t message);
void ReceiveAccelSerial(void);
int16_t GetxAccel(void);
int16_t GetyAccel(void);
int16_t GetRawyAccel(void);
int16_t GetRawxAccel(void);

#endif /* Serial_H */

