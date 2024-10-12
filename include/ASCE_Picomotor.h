#include "hardware/pwm.h" 

#define left_top_motor_p_pin 15 //Each driven @ 14.4 volts 
#define left_top_motor_n_pin 16
#define left_top_motor_PWM_pin 2//PWM1A

#define left_bot_motor_p_pin 17
#define left_bot_motor_n_pin 18
#define left_bot_motor_PWM_pin 3//PWM1B


#define right_top_motor_p_pin 19
#define right_top_motor_n_pin 20
#define right_top_motor_PWM_pin 4//PWM2A

#define right_bot_motor_p_pin 21
#define right_bot_motor_n_pin 22
#define right_bot_motor_PWM_pin 5//PWM2B


#define arm_rotation_motor_p_pin 6  //PWM3A
#define arm_rotation_motor_n_pin 7  //PWM3B
#define arm_extension_motor_p_pin 8 //PWM4A
#define arm_extension_motor_n_pin 9 //PWM4B
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

    int left_top_motor_target=0;
    int left_bot_motor_target=0;
    int right_top_motor_target=0;
    int right_bot_motor_target=0;
    float ease_value=0; //thinking of chaging the name of the interpolation factor/coeff to just "ease value"

    int arm_rotation_speed=0;
    uint8_t arm_servo_pose=0;
    int elbow_servo_pose=0;
    int arm_extension_speed=0;
    uint8_t pitch_servo_pose=0;
    uint8_t yaw_servo_pose=0;
    uint8_t grip_servo_pose=0;
};


#define system_freq 125e6
#define motor_freq 80e3f
#define MG996r_ferq 50.0f //50-60 ig ?
#define SG90_freq 50.0f

#define MG996r_maximum_pulse_width 2500 //in µs
#define MG996r_minimum_pulse_width 500

#define SG90_maximum_pulse_width 2500
#define SG90_minimum_pulse_width 500


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

typedef struct pwm_divisor
{
    float clk_div;
    float wrap;
};

bool calculate_PWM_div(pwm_divisor* buffer, double target_freq)
{
	bool dirtybit=1;
	if (system_freq / (target_freq * 65535) < 1)
	{
		buffer->clk_div = 1.0f;
		buffer->wrap = system_freq / target_freq;
	}
	else
	{
		buffer->clk_div= system_freq / (target_freq * 65535);
		buffer->wrap = 65535;
	}
	if(buffer->wrap<1)
	{
		buffer->wrap = 1;
		dirtybit = 0;
	}else
	if(buffer->wrap > 65535)
	{
		buffer->wrap = 65535;
		dirtybit = 0;
	}
	if(buffer->clk_div>255.7f)
	{
		buffer->clk_div = 255.7f;
		dirtybit = 0;
	}else
	if(buffer->clk_div < 1.0f)
	{
		buffer->clk_div = 1.0f;
	};
	return dirtybit;
}
