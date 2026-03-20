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
#define MAX_PWM 16300
#define WHEEL_BASE 0.127f  // 5 inches ish

uint8_t rx_data[PACKET_SIZE];
float linear_vel, angular_vel;


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
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // LR
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // LL
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // RR
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // RL
    uint32_t duty = htim3.Init.Period / 2;

    // Initialize all to 0 
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    while (1) {
        int16_t left_count  = __HAL_TIM_GET_COUNTER(&htim1);
        int16_t right_count = __HAL_TIM_GET_COUNTER(&htim4);

        // printf("R:%d, L:%d\r\n", right_count, left_count);
        HAL_Delay(500);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        rx_data[i] = rx_data[i] & 0x7F;
    }
    sscanf((char*)rx_data, "%f,%f", &linear_vel, &angular_vel);

    /* set motors */
    float left_vel  = linear_vel - (angular_vel * WHEEL_BASE / 2.0f);
    float right_vel = linear_vel + (angular_vel * WHEEL_BASE / 2.0f);
    printf("left_vel:%f, right_vel:%f\r\n", left_vel, right_vel);
    set_motor(left_vel, right_vel);

    HAL_UART_Receive_IT(&huart6, rx_data, PACKET_SIZE);
}

void set_motor(float left_vel, float right_vel)
{
    int left_pwm  = (int)(fabsf(left_vel)  / MAX_VEL * MAX_PWM);
    int right_pwm = (int)(fabsf(right_vel) / MAX_VEL * MAX_PWM);

    // clamp
    left_pwm  = left_pwm  > MAX_PWM ? MAX_PWM : left_pwm;
    right_pwm = right_pwm > MAX_PWM ? MAX_PWM : right_pwm;

    // left motor
    if (right_vel >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pwm);  // LL (forward)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);          // LR (reverse)
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
    }

    // right motor
    if (right_vel >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, left_pwm); // RL (forward)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);          // RR (reverse)
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, left_pwm);
    }
}

