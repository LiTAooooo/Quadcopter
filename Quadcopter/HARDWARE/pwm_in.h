#ifndef _PWM_IN_H_
#define _PWM_IN_H_

#include "sys.h"

void PWM_IN_Init(void);

extern u16 RC_PWM_IN[4];
void Rc_PWM_To_Angle(void);

#endif
