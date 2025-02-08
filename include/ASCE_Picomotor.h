#include "hardware/pwm.h" 

#define left_top_motor_p_pin 16 //Each driven @ 14.4 volts 
#define left_top_motor_n_pin 15
#define left_top_motor_PWM_pin 2//PWM1A

#define left_bot_motor_p_pin 17
#define left_bot_motor_n_pin 18
#define left_bot_motor_PWM_pin 3//PWM1B


#define right_top_motor_p_pin 20
#define right_top_motor_n_pin 19
#define right_top_motor_PWM_pin 4//PWM2A

#define right_bot_motor_p_pin 22
#define right_bot_motor_n_pin 21
#define right_bot_motor_PWM_pin 5//PWM2B


#define arm_rotation_motor_p_pin 8  //PWM3A
#define arm_rotation_motor_n_pin 9 //PWM3B
#define arm_extension_motor_p_pin 6//PWM4A
#define arm_extension_motor_n_pin 7//PWM4B
#define arm_servo_pin 10           //PWM5A MG996r Check in the datasheet for the servo timings and frequencies 
#define elbow_servo_pin 11         //PWM5B (if different properties put on PWM0A in another pin) MG996r //These two may have different pwm properties but it is to be tested
#define pitch_servo_pin 12         //PWM6A SG90
#define yaw_servo_pin 13           //PWM6B SG90
#define grip_servo_pin 14          //PWM7A SG90


pwm_config left_side_motor_pwm_config; //the slice of the pwm controller for this side is PWM1 
pwm_config right_side_motor_pwm_config; //the slice of the pwm controller for this side is PWM2

pwm_config arm_rotation_pwm_config;   //Slice PWM3
pwm_config arm_extension_pwm_config;  //Slice PWM4

pwm_config arm_elbow_servo_pwm_config; //Slice PWM5
pwm_config pitch_yaw_servo_pwm_config; //Slice PWM6
pwm_config grip_servo_pwm_config;      //Slice PWM7 can be upgraded with 4 more servos or motors ig ? but pin configs must change so

#define left_side_motor_pwm_slice 1
#define right_side_motor_pwm_slice 2
#define arm_rotation_pwm_slice 3
#define arm_extension_pwm_slice 4
#define arm_elbow_servo_pwm_slice 5
#define pitch_yaw_servo_pwm_slice 6
#define grip_servo_pwm_slice 7

struct control_frame
{
  public:
    uint8_t flag_set;

    float motor_bias=0;
    float motor_power=0;

    float arm_rotation_speed=0;
    uint8_t arm_servo_pose=90;
    uint8_t elbow_servo_pose=90;
    float arm_extension_speed=0;
    uint8_t pitch_servo_pose=90;
    uint8_t yaw_servo_pose=90;
    uint8_t grip_servo_pose=0;

};


#define system_freq 125e6
#define motor_freq 30e3f
#define MG996r_ferq 50//50-60 ig ?
#define SG90_freq 50

#define MG996r_maximum_pulse_width 2500 //in µs
#define MG996r_minimum_pulse_width 500

#define SG90_maximum_pulse_width 2500
#define SG90_minimum_pulse_width 500

// Define PWM frequency limits
#define PWM_FREQ_MIN 5000   // Minimum PWM frequency (5 kHz) for max torque
#define PWM_FREQ_MAX 18000  // Maximum PWM frequency (25 kHz) for efficiency
#define SYS_CLOCK 125000000 // Pico system clock (125 MHz)


double mapRange(double a1,double a2,double b1,double b2,double s) 
{
  return b1 + (s-a1)*(b2-b1)/(a2-a1);
}

double constrain(double x, double a, double b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

// Define the deadzone for the rotation joystick (bias)
#define JOYSTICK_DEADZONE 0.15f

// Helper function to apply deadzone to the rotation (bias) input.
// If the absolute value of input is below deadzone, return 0.
// Otherwise, scale it so that the output “starts” at zero after the deadzone.
float apply_deadzone(float input, float deadzone) {
    if (fabsf(input) < deadzone)
        return 0.0f;
    // Scale the input so that it ranges from 0 to 1 (or -1 to 0) outside the deadzone.
    if (input > 0)
        return (input - deadzone) / (1.0f - deadzone);
    else
        return (input + deadzone) / (1.0f - deadzone);
}

static inline uint16_t get_pwm_wrap(uint slice_num) {
    return pwm_hw->slice[slice_num].top;
}

struct pwm_divisor {
    float clk_div;
    uint16_t wrap;
};

bool calculate_PWM_div(pwm_divisor* buffer, double target_freq) {
    bool dirtybit = true;

    // --- Enforce minimum PWM frequency: 5 kHz ---
    if (target_freq < PWM_FREQ_MIN) {
        target_freq = PWM_FREQ_MIN;
        dirtybit = false;
    }

    // --- Enforce a minimum resolution (wrap+1 >= 1000) ---
    // For a given target frequency and a given divider, the ideal wrap is:
    //    wrap = (system_freq / (clk_div * target_freq)) - 1.
    // To maximize resolution we try with the smallest divider: clk_div = 1.
    double ideal_wrap = system_freq / target_freq - 1;
    
    // If the ideal wrap is less than 1000, resolution is too low.
    // In that case, we must increase the divider so that wrap becomes at least 1000.
    if (ideal_wrap < 1000) {
        // Set the resolution to 1000 (i.e. 1001 counts per cycle)
        buffer->wrap = 1000;
        // Calculate the divider needed to achieve that resolution at the target frequency.
        // From: system_freq / (clk_div * (wrap+1)) = target_freq
        buffer->clk_div = system_freq / (target_freq * (buffer->wrap + 1));
        dirtybit = false;
    } 
    // If the ideal wrap exceeds the maximum allowed value, clamp it.
    else if (ideal_wrap > 65534) {
        buffer->wrap = 65534;
        buffer->clk_div = system_freq / (target_freq * (buffer->wrap + 1));
        dirtybit = false;
    } 
    // Otherwise, use the ideal settings with the minimum divider (1.0f)
    else {
        buffer->clk_div = 1.0f;
        buffer->wrap = (uint16_t)(ideal_wrap);
    }

    // --- Clamp the clock divider to its allowed range ---
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