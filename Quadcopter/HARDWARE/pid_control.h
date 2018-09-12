#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "sys.h"
#include "data_struct.h"

void PID_Init(void);
void PID_Position_Cal(struct _pid_control_ * PID, float Expect, float Measure);
void Pid_Inner_Loop(void);
void Pid_Outer_Loop(void);
#endif
