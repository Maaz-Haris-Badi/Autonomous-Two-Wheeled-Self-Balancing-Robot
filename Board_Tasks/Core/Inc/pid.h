#ifndef PID_H
#define PID_H

#include "config.h"

float Balance_PID_Compute(BalancePID_t *pid, float measured_angle, float dt);
uint8_t Is_Fallen(float angle);

#endif /* PID_H */