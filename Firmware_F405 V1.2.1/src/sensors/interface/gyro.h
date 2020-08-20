#ifndef __GYRO_H
#define __GYRO_H

#include "sys.h"
#include "stabilizer_types.h"

#define CALIBRATING_GYRO_CYCLES         1000

extern Axis3i16 gyroADC;

bool gyroIsCalibrationComplete(void);
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
bool gyroInit(float gyroUpdateRate);
void gyroUpdate(Axis3f *gyro);




#endif
