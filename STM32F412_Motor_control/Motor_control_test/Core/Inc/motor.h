#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define NUM_MOTORS 3
#define MAX_PWM_ARR 800      // Matches TIM2 ARR (799 + 1)
#define MAX_CONTROL_PWM 300// Clamps output to ~30% power for low RPM

typedef struct {
    // Hardware PWM & Direction
    TIM_HandleTypeDef *htim_pwm;
    uint32_t pwm_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    bool invert_dir;

    // Hardware Encoder
    TIM_HandleTypeDef *htim_enc;
    bool invert_enc;
    int32_t encoder_pos;
    int16_t last_raw_cnt;

    // Position PID Tuning Parameters
    float kp;
    float kd;
    int32_t target_pos;
    int32_t prev_error;
} Motor_t;

void Motors_Init(Motor_t *m);
void Motor_UpdateEncoder(Motor_t *m);
void Motor_SetRawOutput(Motor_t *m, int16_t command);
bool Motor_UpdatePositionPID(Motor_t *m);

#endif /* INC_MOTOR_H_ */
