#include "picomotor.h"
#include <math.h>

// Utility functions for mapping and constraining values

pwm_config left_side_motor_pwm_config;
pwm_config right_side_motor_pwm_config;
pwm_config arm_rotation_pwm_config;
pwm_config arm_extension_pwm_config;
pwm_config arm_elbow_servo_pwm_config;
pwm_config pitch_yaw_servo_pwm_config;
pwm_config grip_servo_pwm_config;

double mapRange(double a1, double a2, double b1, double b2, double s) {
    return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}

double constrain(double x, double a, double b) {
    if (x < a) {
        return a;
    } else if (b < x) {
        return b;
    } else {
        return x;
    }
}

float apply_deadzone(float input, float deadzone) {
    if (fabsf(input) < deadzone)
        return 0.0f;
    if (input > 0)
        return (input - deadzone) / (1.0f - deadzone);
    else
        return (input + deadzone) / (1.0f - deadzone);
}

uint16_t get_pwm_wrap(uint slice_num) {
    return pwm_hw->slice[slice_num].top;
}

bool calculate_PWM_div(pwm_divisor* buffer, double target_freq) {
    bool dirtybit = true;

    if (target_freq < PWM_FREQ_MIN) {
        target_freq = PWM_FREQ_MIN;
        dirtybit = false;
    }

    double ideal_wrap = system_freq / target_freq - 1;

    if (ideal_wrap < 1000) {
        buffer->wrap = 1000;
        buffer->clk_div = system_freq / (target_freq * (buffer->wrap + 1));
        dirtybit = false;
    } else if (ideal_wrap > 65534) {
        buffer->wrap = 65534;
        buffer->clk_div = system_freq / (target_freq * (buffer->wrap + 1));
        dirtybit = false;
    } else {
        buffer->clk_div = 1.0f;
        buffer->wrap = (uint16_t)(ideal_wrap);
    }

    if (buffer->clk_div < 1.0f) {
        buffer->clk_div = 1.0f;
        dirtybit = false;
    }
    if (buffer->clk_div > 255.7f) {
        buffer->clk_div = 255.7f;
        dirtybit = false;
    }

    return dirtybit;
}

