#ifndef _IMU_H_
#define _IMU_H_
#include "data_struct.h"

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz);
void Get_Eulerian_Angle(struct _out_angle *angle);

#endif
