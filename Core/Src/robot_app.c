#include "robot_app.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal_uart.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart6;

#define PACKET_SIZE 14
#define MAX_VEL 0.5f
#define MAX_PWM 65000
#define WHEEL_BASE 0.145f

#define LOOP_MS 20
#define TICKS_PER_REV 780.0f
#define WHEEL_RADIUS 0.022f

uint8_t rx_data[PACKET_SIZE];
float linear_vel, angular_vel;
float left_vel_target = 0.0f, right_vel_target = 0.0f;
double theta = 0.0; double x_pos = 0.0; double y_pos = 0.0;

PI_Controller left_pid  = {0.75f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f};
PI_Controller right_pid = {0.75f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void set_motor(float left_vel, float right_vel);

float pi_control_compute(PI_Controller *pid, float target, float actual, float dt)
{
    float error = target - actual;

    if (fabsf(target) < 0.01f) {
        pid->integral = 0.0f;
        pid->prev_err = 0.0f;
        return 0.0f;
    }

    pid->integral += error * dt;
    if (pid->integral >  2.0f) pid->integral =  2.0f;
    if (pid->integral < -2.0f) pid->integral = -2.0f;

    pid->prev_err = error;

    return (pid->kp * error) + (pid->ki * pid->integral);
}

void my_robot_app()
{
    HAL_UART_Receive_IT(&huart6, rx_data, PACKET_SIZE);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    while (1) {
        int16_t left_diff  = __HAL_TIM_GET_COUNTER(&htim1);
        int16_t right_diff = __HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);

        float dt = LOOP_MS / 1000.0f;
        float left_vel_actual  = (left_diff  / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS) / dt;
        float right_vel_actual = (right_diff / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS) / dt;

        // ---- ODOMETRY -----
        float robot_v = (right_vel_actual + left_vel_actual) / 2.0f;
        float robot_w = (right_vel_actual - left_vel_actual) / WHEEL_BASE;

        char tx_buf[64];
        int len = snprintf(tx_buf, sizeof(tx_buf), "V,%.4f,%.4f\r\n", robot_v, robot_w);

        HAL_UART_Transmit_IT(&huart6, (uint8_t*)tx_buf, len);
        // ---- ODOMETRY -----

        float left_output  = pi_control_compute(&left_pid,  left_vel_target,  left_vel_actual,  dt);
        float right_output = pi_control_compute(&right_pid, right_vel_target, right_vel_actual, dt);

        set_motor(left_output, right_output);

        // DEBUG FOR PID TUNING, BUT NOT NECCESSARY ANYMORE
        // char tx_buf[64];
        // int len = snprintf(tx_buf, sizeof(tx_buf), "%f,%f,%f,%f\r\n",
        //                    left_vel_target, left_vel_actual,
        //                    right_vel_target, right_vel_actual);
        // HAL_UART_Transmit(&huart6, (uint8_t*)tx_buf, len, 10);

        HAL_Delay(LOOP_MS);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        rx_data[i] = rx_data[i] & 0x7F;
    }
    sscanf((char*)rx_data, "%f,%f", &linear_vel, &angular_vel);

    if (linear_vel >  0.7f) linear_vel =  0.7f;
    if (linear_vel < -0.7f) linear_vel = -0.7f;

    left_vel_target  = linear_vel - (angular_vel * WHEEL_BASE / 2.0f);
    right_vel_target = linear_vel + (angular_vel * WHEEL_BASE / 2.0f);

    HAL_UART_Receive_IT(&huart6, rx_data, PACKET_SIZE);
}

void set_motor(float left_vel, float right_vel)
{
    if (fabsf(left_vel)  < 0.01f) left_vel  = 0.0f;
    if (fabsf(right_vel) < 0.01f) right_vel = 0.0f;

    int left_pwm  = (int)(fabsf(left_vel)  / MAX_VEL * MAX_PWM);
    int right_pwm = (int)(fabsf(right_vel) / MAX_VEL * MAX_PWM);

    left_pwm  = left_pwm  > MAX_PWM ? MAX_PWM : left_pwm;
    right_pwm = right_pwm > MAX_PWM ? MAX_PWM : right_pwm;

    if (left_vel >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, left_pwm);
    }

    if (right_vel >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, right_pwm);
    }
}