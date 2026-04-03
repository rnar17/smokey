#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include "main.h"

typedef struct {
    float kp, ki, kd;
    float integral, deriv, prev_err;
} PI_Controller;

void my_robot_app(void);
void set_motor(float left_vel, float right_vel);
float pi_control_compute(PI_Controller *pid, float target, float actual, float dt);

#endif