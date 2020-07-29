#include "pid.h"

float pid_calculate(PIDProfile * profile, float input, float setpoint, float delta_time) {

  if(profile == NULL || delta_time == 0) {
    return 0;
  }

  float error  = setpoint - input;
  float p_out  = error * profile->kP;

  profile->integral += error * delta_time;
  if(profile->integral >  profile->max_output) profile->integral =  profile->max_output; //clamping
  if(profile->integral < -profile->max_output) profile->integral = -profile->max_output;
  float i_out = profile->integral * profile->kI;

  float d_out = ((error - profile->stored_error) / delta_time) * profile->kD;
  profile->stored_error = error;

  float output = p_out + i_out + d_out;
  if(output >  profile->max_output) output =  profile->max_output;
  if(output < -profile->max_output) output = -profile->max_output;

  return output;
}
void  pid_clear(PIDProfile * profile) {
  profile->integral = 0;
  profile->stored_error = 0;
}
