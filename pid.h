#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {

  float kP;
  float kI;
  float kD;

  float max_output;

  float integral;
  float stored_error;

} PIDProfile;

float pid_calculate(PIDProfile * profile, float input, float setpoint, float delta_time);
void  pid_clear(PIDProfile * profile);


#endif
