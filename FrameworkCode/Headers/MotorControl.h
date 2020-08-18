/****************************************************************************

  Header file for motor control module
  
 ****************************************************************************/

#ifndef MotorControl_H
#define MotorControl_H

#include "ES_Types.h"

// Motor control definitions
#define FULL_SPEED_DC 100
#define HALF_SPEED_DC 50
#define STOP_DC 0

#define FWD 0 // Forward designator
#define REV 1 // Reverse designator
#define CW  0 // Clockwise designator
#define CCW 1 // Counterclockwise designator
#define CONTINUOUS 0 // Continuous rotation designator

// Public Function Prototypes

void InitMotorControl(void);
void StopMotors(void);
void Rotate(uint8_t direction);
void DriveStraight(uint8_t direction);

void SetMotorADC(uint8_t DC);
void SetMotorBDC(uint8_t DC);

#endif /* MotorControl_H */
