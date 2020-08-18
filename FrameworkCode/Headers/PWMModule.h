/****************************************************************************

  Header file for the PWM Module

 ****************************************************************************/

#ifndef PWMModule_H
#define PWMModule_H

#include "ES_Types.h"

// Pin designation
#define PWMA  0
#define PWMB  1

// Public Function Prototypes

bool InitPWMModule(void);
void PWM_SetDuty(uint32_t DutyCycle, uint8_t Pin);

#endif /* PWMModule_H */

