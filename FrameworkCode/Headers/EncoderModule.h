/****************************************************************************

  Header file for the encoder module
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef EncoderModule_H
#define EncoderModule_H

#include "ES_Types.h"

// Public Function Prototypes
void InitEncoderCapture(void);

void LeftEncoderCaptureResponse(void);
void RightEncoderCaptureResponse(void);

void InitPeriodicInt(void);
void PeriodicControlResponse(void);

void SetTargetRightDC(uint8_t NewTarget);
void SetTargetLeftDC(uint8_t NewTarget);

void SetEncoderDistanceMM(uint16_t travel);
void SetEncoderDistanceDeg(uint16_t deg);

void ResetEncoderCount(void);
void DisablePositionTracking(void);

void DecrementEncoderCount(void);
void IncrementEncoderCount(void);

int32_t GetEncoderCountLeft(void);
int32_t GetEncoderCountRight(void);

#endif /* EncoderModule_H */
