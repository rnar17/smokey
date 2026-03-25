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
#define MAX_PWM 50000
#define WHEEL_BASE 0.127f  // 5 inches ish

#define LOOP_MS 20
#define TICKS_PER_REV 780.0f
#define WHEEL_RADIUS 0.022f

uint8_t rx_data[PACKET_SIZE];
float linear_vel, angular_vel;
float left_vel_target = 0.0f, right_vel_target = 0.0f;
float prev_err =0.0f;

PI_Controller left_pid = {2.0f, 0.75, 0.001f};
PI_Controller right_pid = {2.0f, 0.75, 0.001f};

float pi_control_compute(PI_Controller *pid, float target, float actual, float dt)
{
    float error = target - actual;

    pid->deriv= 0;//(error - prev_err) / dt;
    if (target==0.0f) {
        pid->integral = 0.0f;
    } else {
        pid->integral += error * dt;
        if (pid->integral > 1.0f) pid->integral = 1.0f;
        if (pid->integral < -1.0f) pid->integral = -1.0f;
    }
    prev_err = error;
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->deriv * pid->kd);
}

/* STM32 to Pi Communication Sanity Check */
// void my_robot_app(void)
// {
//     while (1) {
//         uint8_t msg[] = "hello\r\n";
//         HAL_UART_Transmit(&huart6, msg, sizeof(msg)-1, HAL_MAX_DELAY);
//         HAL_Delay(500);
//     }    
// }

/* Pi to STM32 Communication Sanity Check */
// void my_robot_app(void)
// {
//     HAL_UART_Receive_IT(&huart6, rx_data, n);
//     while (1) {
//     }
// }
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     // for (uint8_t i = 1; i < n; i++) {
//     //     rx_data[i] = rx_data[i] & 0x7F;
//     // }
//     rx_data[0] = rx_data[0] & 0x7F;
//     HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, rx_data, n, 10);
//     HAL_UART_Receive_IT(&huart6, rx_data, n);
// }

/* Pi Python script from pi to STM32 Check */
void my_robot_app() 
{
    HAL_UART_Receive_IT(&huart6, rx_data, PACKET_SIZE);

    // Start encoder timers
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // Start PWM timers
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // R LPWM (reverse)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // R RPWM (forward)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // L LPWM (reverse)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // L RPWM (forward)

    // Initialize all to 0 
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
        float left_vel_actual = (left_diff / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS) / dt;
        float right_vel_actual = (right_diff / TICKS_PER_REV) * (2.0f * M_PI * WHEEL_RADIUS) / dt;

        float left_output = pi_control_compute(&left_pid, left_vel_target, left_vel_actual, dt);
        float right_output = pi_control_compute(&right_pid, right_vel_target, right_vel_actual, dt);

        set_motor(left_output, right_output);

        printf("%f, %f, %f, %f\r\n", left_vel_target, left_vel_actual, right_vel_target, right_vel_actual);

        // char tx_buf[32];
        // int len = snprintf(tx_buf, sizeof(tx_buf), "%d,%d\r\n", left_diff, right_diff);
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
    linear_vel  = linear_vel  > 0.7f ? 0.7f : linear_vel;
    /* set motors */
    left_vel_target  = linear_vel - (angular_vel * WHEEL_BASE / 2.0f);
    right_vel_target = linear_vel + (angular_vel * WHEEL_BASE / 2.0f);

    HAL_UART_Receive_IT(&huart6, rx_data, PACKET_SIZE);
}

void set_motor(float left_vel, float right_vel)
{
    int left_pwm  = (int)(fabsf(left_vel)  / MAX_VEL * MAX_PWM);
    int right_pwm = (int)(fabsf(right_vel) / MAX_VEL * MAX_PWM);

    // clamp
    left_pwm  = left_pwm  > MAX_PWM ? MAX_PWM : left_pwm;
    right_pwm = right_pwm > MAX_PWM ? MAX_PWM : right_pwm;

    // left motor (CH3 = LPWM, CH4 = RPWM)
    if (left_vel >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, left_pwm);   // RPWM (forward)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);           // LPWM (reverse)
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, left_pwm);
    }

    // right motor (CH1 = LPWM, CH2 = RPWM)
    if (right_vel >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pwm);  // RPWM (forward)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);           // LPWM (reverse)
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, right_pwm);
    }
}

