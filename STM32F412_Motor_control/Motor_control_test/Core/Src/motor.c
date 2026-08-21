#include "motor.h"
#include <stdlib.h>

void Motors_Init(Motor_t *m) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Start PWM generation
        HAL_TIM_PWM_Start(m[i].htim_pwm, m[i].pwm_channel);

        // Start hardware encoder counter
        HAL_TIM_Encoder_Start(m[i].htim_enc, TIM_CHANNEL_ALL);

        // Reset state
        __HAL_TIM_SET_COUNTER(m[i].htim_enc, 0);
        m[i].last_raw_cnt = 0;
        m[i].encoder_pos = 0;
        m[i].target_pos = 0;
        m[i].prev_error = 0;

        Motor_SetRawOutput(&m[i], 0);
    }
}

void Motor_UpdateEncoder(Motor_t *m) {
    int16_t current_raw = (int16_t)__HAL_TIM_GET_COUNTER(m->htim_enc);
    int16_t delta = current_raw - m->last_raw_cnt;
    m->last_raw_cnt = current_raw;

    if (m->invert_enc) {
        delta = -delta;
    }
    m->encoder_pos += delta;
}

void Motor_SetRawOutput(Motor_t *m, int16_t command) {
    bool dir = (command >= 0);
    if (m->invert_dir) dir = !dir;

    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

    int32_t speed = abs(command);
    if (speed > MAX_PWM_ARR) speed = MAX_PWM_ARR;

    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, (uint32_t)speed);
}

bool Motor_UpdatePositionPID(Motor_t *m) {
    Motor_UpdateEncoder(m);

    int32_t error = m->target_pos - m->encoder_pos;
    int32_t derivative = error - m->prev_error;
    m->prev_error = error;

    // Deadband check: if within +/- 3 ticks, stop motor completely
    if (abs(error) <= 3) {
        Motor_SetRawOutput(m, 0);
        return true;
    }

    float output = (m->kp * (float)error) + (m->kd * (float)derivative);

    // Minimum power kick to overcome gearbox static friction
    int16_t min_pwm = 80; // ~10% minimum power floor
    if (output > 0 && output < min_pwm) output = min_pwm;
    if (output < 0 && output > -min_pwm) output = -min_pwm;

    // Clamp maximum power
    if (output > MAX_CONTROL_PWM) output = MAX_CONTROL_PWM;
    if (output < -MAX_CONTROL_PWM) output = -MAX_CONTROL_PWM;

    Motor_SetRawOutput(m, (int16_t)output);
    return false;
}
