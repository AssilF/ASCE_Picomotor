#include <stdio.h>
#include "pico/stdlib.h"
#include <include/ASCE_Picomotor.h>
#include "pico/i2c_slave.h"
#include "Wire.h"
#include "inttypes.h"
#include "string.h"
#include "hardware/pwm.h"
#include <math.h>
#include "hardware/timer.h"
#include "pico/time.h"
#include "pico/multicore.h"


#define secure_action_timeout 200

int speed;
int left_top_motor_speed;
int left_bot_motor_speed;
int right_top_motor_speed;
int right_bot_motor_speed;

uint16_t wrap_set[8]; //each slice has its own wrap

control_frame pico_frame;

char frame_buffer[sizeof(pico_frame)];

/*flags,LTmot,LBmot,RTmot,RBmot,Intrp,ArmRotation,ArmPose,ElbowPose,Extension,Pitch,Yaw,Grip*/

char callback[] = "ba3lkhir ESP32";

void answer()
{
    for(int i; i<sizeof(callback);i++)
    {
    Wire.write((uint8_t)callback[i]);
    }
}

uint32_t coms_time_stamp;
int fetch_index=0;

void fetch_command(int num)
{
        while(Wire.available()){
        frame_buffer[fetch_index]=Wire.read(); //coms indicator
        fetch_index++;
        if(fetch_index>=sizeof(frame_buffer))
        {   
        fetch_index=0;
        memcpy(&pico_frame,&frame_buffer,sizeof(pico_frame));
        }
        }
        gpio_put(25,!gpio_get_out_level(25));
        coms_time_stamp=to_ms_since_boot(get_absolute_time()); //reset to defaults if coms timeout after 200ms
}

void unpack_frame()
{ //add further treatment like mapping signs and such
    printf("The packets fetched:\nflags:%x\nMotor Power:%f\nMotor Bias:%f\nRot Speed:%f\nArm pose:%i\nElbow pose:%i\nExtns Speed:%f\nPitch pose:%i\nYaw pose:%i\nGrip pose:%i\n\n\n",
    pico_frame.flag_set,pico_frame.motor_power,pico_frame.motor_bias,pico_frame.arm_rotation_speed,pico_frame.arm_servo_pose,pico_frame.elbow_servo_pose,
    pico_frame.arm_extension_speed,pico_frame.pitch_servo_pose,pico_frame.yaw_servo_pose,pico_frame.grip_servo_pose);
}

uint32_t pwm_reference_count;


struct Servo
{
    int pin;
    float max_angle=180;
    int maximum=2000;
    int minimum=1000;
    int pulse_width=minimum;
    Servo(){}
    Servo(int pin,int min, int max)
    {
        this->maximum = max;
        this->minimum = min;
        this->pin = pin;
        gpio_init(this->pin);
        gpio_set_dir(this->pin,1);
    }
    Servo(int pin,int min, int max,float max_angle)
    {
        this->maximum = max;
        this->minimum = min;
        this->pin = pin;
        this->max_angle = max_angle;
        gpio_init(this->pin);
        gpio_set_dir(this->pin,1);
    }
    void setPose(float angle)
    {
        pulse_width=mapRange(0,this->max_angle,this->minimum,this->maximum,angle);
    }
    void compare_instance(int instance_of_time) // i know bad optimization but I am working without a plan and out of despair bruh
    {
        instance_of_time>this->pulse_width? gpio_put(this->pin,0):gpio_put(this->pin,1);
    }
    void init(int pin,int min, int max)
    {
        this->maximum = max;
        this->minimum = min;
        this->pin = pin;
        this->max_angle = max_angle;
        gpio_init(this->pin);
        gpio_set_dir(this->pin,1);
    }
    void init(int pin,int min, int max,float max_angle)
    {
        this->maximum = max;
        this->minimum = min;
        this->pin = pin;
        this->max_angle = max_angle;
        gpio_init(this->pin);
        gpio_set_dir(this->pin,1);
    }
};

struct ASCE_Mechiane
{
    Servo arm;
    Servo elbow;
    Servo pitch;
    Servo yaw;
    Servo grip;
    
    void init()
    {
        this->arm.init(arm_servo_pin,400,2400);
        this->elbow.init(elbow_servo_pin,400,2400);
        this->pitch.init(pitch_servo_pin,1000,2000);
        this->yaw.init(yaw_servo_pin,1000,2000);
        this->grip.init(grip_servo_pin,1000,2000);
    }
    void run_engine(int instance_of_time)
    {
        this->arm.compare_instance(instance_of_time);
        this->elbow.compare_instance(instance_of_time);
        this->pitch.compare_instance(instance_of_time);
        this->yaw.compare_instance(instance_of_time);
        this->grip.compare_instance(instance_of_time);
    }
};

ASCE_Mechiane Mechiane;

bool repeating_timer_callback(repeating_timer *t) {
    pwm_reference_count+=5;    
    if(pwm_reference_count>20000){pwm_reference_count=0;};//cap em frequencies at 20ms

    Mechiane.run_engine(pwm_reference_count);    

    return true;
}

void init_coms()
{

}

void secure_action_check()
{
   

}

void second_core_test()
{
    static int speed=0;
    while(1){
     if(pico_frame.flag_set==1){
            if(pico_frame.motor_power>=0.0)
            {
                gpio_put(left_top_motor_n_pin,0);
                gpio_put(left_top_motor_p_pin,1);
                gpio_put(right_top_motor_n_pin,0);
                gpio_put(right_top_motor_p_pin,1);
                gpio_put(left_bot_motor_n_pin,0);
                gpio_put(left_bot_motor_p_pin,1);
                gpio_put(right_bot_motor_n_pin,0);
                gpio_put(right_bot_motor_p_pin,1);
                speed = (int)(pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
                if(pico_frame.motor_bias>=0.5){
                gpio_put(right_bot_motor_n_pin,!gpio_get_out_level(right_bot_motor_n_pin));
                gpio_put(right_bot_motor_p_pin,!gpio_get_out_level(right_bot_motor_p_pin));
                gpio_put(right_top_motor_n_pin,!gpio_get_out_level(right_top_motor_n_pin));
                gpio_put(right_top_motor_p_pin,!gpio_get_out_level(right_top_motor_p_pin));
                pwm_set_gpio_level(left_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(left_bot_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_bot_motor_PWM_pin,speed);
                }else if(pico_frame.motor_bias<=-0.5)
                {
                gpio_put(left_bot_motor_n_pin,!gpio_get_out_level(left_bot_motor_n_pin));
                gpio_put(left_bot_motor_p_pin,!gpio_get_out_level(left_bot_motor_p_pin));
                gpio_put(left_top_motor_n_pin,!gpio_get_out_level(left_top_motor_n_pin));
                gpio_put(left_top_motor_p_pin,!gpio_get_out_level(left_top_motor_p_pin));
                pwm_set_gpio_level(left_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(left_bot_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_bot_motor_PWM_pin,speed);
                }else
                {
                pwm_set_gpio_level(left_top_motor_PWM_pin,0);
                pwm_set_gpio_level(left_bot_motor_PWM_pin,0);
                pwm_set_gpio_level(right_top_motor_PWM_pin,0);
                pwm_set_gpio_level(right_bot_motor_PWM_pin,0);
                }
            }else
            {
                gpio_put(left_top_motor_n_pin,1);
                gpio_put(left_top_motor_p_pin,0); 
                gpio_put(right_top_motor_n_pin,1);
                gpio_put(right_top_motor_p_pin,0);
                gpio_put(left_bot_motor_n_pin,1);
                gpio_put(left_bot_motor_p_pin,0);
                gpio_put(right_bot_motor_n_pin,1);
                gpio_put(right_bot_motor_p_pin,0);
                speed = (int)(-pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
                if(pico_frame.motor_bias>=0.5){
                gpio_put(right_bot_motor_n_pin,!gpio_get_out_level(right_bot_motor_n_pin));
                gpio_put(right_bot_motor_p_pin,!gpio_get_out_level(right_bot_motor_p_pin));
                gpio_put(right_top_motor_n_pin,!gpio_get_out_level(right_top_motor_n_pin));
                gpio_put(right_top_motor_p_pin,!gpio_get_out_level(right_top_motor_p_pin));
                pwm_set_gpio_level(left_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(left_bot_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_bot_motor_PWM_pin,speed);
                }else if(pico_frame.motor_bias<=-0.5)
                {
                gpio_put(left_bot_motor_n_pin,!gpio_get_out_level(left_bot_motor_n_pin));
                gpio_put(left_bot_motor_p_pin,!gpio_get_out_level(left_bot_motor_p_pin));
                gpio_put(left_top_motor_n_pin,!gpio_get_out_level(left_top_motor_n_pin));
                gpio_put(left_top_motor_p_pin,!gpio_get_out_level(left_top_motor_p_pin));
                pwm_set_gpio_level(left_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(left_bot_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_top_motor_PWM_pin,speed);
                pwm_set_gpio_level(right_bot_motor_PWM_pin,speed);
                }
        }}else{
        if(pico_frame.motor_power>=0.0)
        {
            gpio_put(left_top_motor_n_pin,0);
            gpio_put(left_top_motor_p_pin,1);
            gpio_put(right_top_motor_n_pin,0);
            gpio_put(right_top_motor_p_pin,1);
            gpio_put(left_bot_motor_n_pin,0);
            gpio_put(left_bot_motor_p_pin,1);
            gpio_put(right_bot_motor_n_pin,0);
            gpio_put(right_bot_motor_p_pin,1);
            speed = (int)(pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
            if(pico_frame.motor_bias>=0.0){
            pwm_set_gpio_level(left_top_motor_PWM_pin,speed);
            pwm_set_gpio_level(left_bot_motor_PWM_pin,speed);
            pwm_set_gpio_level(right_top_motor_PWM_pin,speed-speed*(pico_frame.motor_bias));
            pwm_set_gpio_level(right_bot_motor_PWM_pin,speed- speed*(pico_frame.motor_bias));
            }else
            {
            pwm_set_gpio_level(left_top_motor_PWM_pin,speed-speed*(-pico_frame.motor_bias));
            pwm_set_gpio_level(left_bot_motor_PWM_pin,speed-speed*(-pico_frame.motor_bias));
            pwm_set_gpio_level(right_top_motor_PWM_pin,speed);
            pwm_set_gpio_level(right_bot_motor_PWM_pin,speed);
            }
        }else
        {
            gpio_put(left_top_motor_n_pin,1);
            gpio_put(left_top_motor_p_pin,0); 
            gpio_put(right_top_motor_n_pin,1);
            gpio_put(right_top_motor_p_pin,0);
            gpio_put(left_bot_motor_n_pin,1);
            gpio_put(left_bot_motor_p_pin,0);
            gpio_put(right_bot_motor_n_pin,1);
            gpio_put(right_bot_motor_p_pin,0);
            speed = (int)(-pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
            if(pico_frame.motor_bias>=0.0){
            pwm_set_gpio_level(left_top_motor_PWM_pin,speed);
            pwm_set_gpio_level(left_bot_motor_PWM_pin,speed);
            pwm_set_gpio_level(right_top_motor_PWM_pin,speed-speed*(pico_frame.motor_bias));
            pwm_set_gpio_level(right_bot_motor_PWM_pin,speed-speed*(pico_frame.motor_bias));
            }else
            {
            pwm_set_gpio_level(left_top_motor_PWM_pin,speed- speed*(-pico_frame.motor_bias));
            pwm_set_gpio_level(left_bot_motor_PWM_pin,speed- speed*(-pico_frame.motor_bias));
            pwm_set_gpio_level(right_top_motor_PWM_pin,speed);
            pwm_set_gpio_level(right_bot_motor_PWM_pin,speed);
            }
        }
        }
        }

}


int main()
{   
    
    stdio_init_all();

    left_side_motor_pwm_config=pwm_get_default_config();
    right_side_motor_pwm_config=pwm_get_default_config();
    arm_extension_pwm_config=pwm_get_default_config();
    arm_rotation_pwm_config=pwm_get_default_config();

    //calculating divs and wraps:
    pwm_divisor div;

    //lez start configuring this stuff *sigh* (first commit commence here) //default PWM clock speed is about 125MHz so do your math accordingly (I am talking to you my future self)
    calculate_PWM_div(&div,motor_freq);
    pwm_config_set_clkdiv(&left_side_motor_pwm_config,div.clk_div); //this gives us a frequency of 80KHz (aka one wrap each 0.0008). . . but like, hold up, when will the pwm register wrap ? (I beleive the counter buffer is like, 16 bits ? about up to 65,536)
    pwm_config_set_clkdiv(&right_side_motor_pwm_config,div.clk_div);
    pwm_config_set_clkdiv(&arm_extension_pwm_config,div.clk_div);
    pwm_config_set_clkdiv(&arm_rotation_pwm_config,div.clk_div);
    pwm_config_set_wrap(&left_side_motor_pwm_config,div.wrap); //this gives us a frequency of 80KHz (aka one wrap each 0.0008). . . but like, hold up, when will the pwm register wrap ? (I beleive the counter buffer is like, 16 bits ? about up to 65,536)
    pwm_config_set_wrap(&right_side_motor_pwm_config,div.wrap);
    pwm_config_set_wrap(&arm_extension_pwm_config,div.wrap);
    pwm_config_set_wrap(&arm_rotation_pwm_config,div.wrap); 

    //saving the wraps;
    wrap_set[left_side_motor_pwm_slice]=div.wrap;
    wrap_set[right_side_motor_pwm_slice]=div.wrap;
    wrap_set[arm_extension_pwm_slice]=div.wrap;
    wrap_set[arm_rotation_pwm_slice]=div.wrap;;
    printf("calculated motor div: %f and wrap: %i\n",div.clk_div,div.wrap);

    gpio_init(25);
    gpio_set_dir(25,1);

    gpio_init(left_top_motor_n_pin);
    gpio_init(left_top_motor_p_pin);
    gpio_init(left_top_motor_PWM_pin);

    gpio_init(right_top_motor_n_pin);
    gpio_init(right_top_motor_p_pin);
    gpio_init(right_top_motor_PWM_pin);

    gpio_init(left_bot_motor_n_pin);
    gpio_init(left_bot_motor_p_pin);
    gpio_init(left_bot_motor_PWM_pin);

    gpio_init(right_bot_motor_n_pin);
    gpio_init(right_bot_motor_p_pin);
    gpio_init(right_bot_motor_PWM_pin);

    gpio_init(arm_rotation_motor_n_pin);
    gpio_init(arm_rotation_motor_p_pin);

    gpio_init(arm_servo_pin);
    gpio_init(elbow_servo_pin);

    gpio_init(arm_extension_motor_n_pin);
    gpio_init(arm_extension_motor_p_pin);
    gpio_init(pitch_servo_pin);
    gpio_init(yaw_servo_pin);
    gpio_init(grip_servo_pin);

    gpio_set_function(left_top_motor_PWM_pin,GPIO_FUNC_PWM);
    gpio_set_function(left_bot_motor_PWM_pin,GPIO_FUNC_PWM);
    gpio_set_function(right_top_motor_PWM_pin,GPIO_FUNC_PWM);
    gpio_set_function(right_bot_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_set_function(arm_rotation_motor_n_pin,GPIO_FUNC_PWM);
    gpio_set_function(arm_rotation_motor_p_pin,GPIO_FUNC_PWM);
    gpio_set_function(arm_extension_motor_n_pin,GPIO_FUNC_PWM);
    gpio_set_function(arm_extension_motor_p_pin,GPIO_FUNC_PWM);


    gpio_set_dir(left_top_motor_n_pin,1);
    gpio_set_dir(left_top_motor_p_pin,1);
    gpio_set_dir(left_top_motor_PWM_pin,1);

    gpio_set_dir(right_top_motor_n_pin,1);
    gpio_set_dir(right_top_motor_p_pin,1);
    gpio_set_dir(right_top_motor_PWM_pin,1);

    gpio_set_dir(left_bot_motor_n_pin,1);
    gpio_set_dir(left_bot_motor_p_pin,1);
    gpio_set_dir(left_bot_motor_PWM_pin,1);

    gpio_set_dir(right_bot_motor_n_pin,1);
    gpio_set_dir(right_bot_motor_p_pin,1);
    gpio_set_dir(right_bot_motor_PWM_pin,1);

    gpio_set_dir(arm_rotation_motor_n_pin,1);
    gpio_set_dir(arm_rotation_motor_p_pin,1);

    gpio_set_dir(arm_extension_motor_n_pin,1);
    gpio_set_dir(arm_extension_motor_p_pin,1);


    pwm_init(left_side_motor_pwm_slice,&left_side_motor_pwm_config,0);
    pwm_init(right_side_motor_pwm_slice,&right_side_motor_pwm_config,0);
    pwm_init(arm_extension_pwm_slice,&arm_extension_pwm_config,0);
    pwm_init(arm_rotation_pwm_slice,&arm_rotation_pwm_config,0);
    pwm_init(pitch_yaw_servo_pwm_slice,&pitch_yaw_servo_pwm_config,0);
    pwm_init(grip_servo_pwm_slice,&grip_servo_pwm_config,0);



    gpio_set_dir(arm_servo_pin,1); 
    gpio_set_dir(elbow_servo_pin,1);
    gpio_set_dir(pitch_servo_pin,1);
    gpio_set_dir(yaw_servo_pin,1);
    gpio_set_dir(grip_servo_pin,1);

    gpio_init(0);
    gpio_set_function(0,GPIO_FUNC_I2C);
    gpio_pull_up(0);

    gpio_init(1);
    gpio_set_function(1,GPIO_FUNC_I2C);
    gpio_pull_up(1);

    i2c_init(i2c0,400*1000);

    Wire.onReceive(fetch_command);
    Wire.onRequest(answer);
    Wire.begin(0x17);
    
    pwm_set_enabled(left_side_motor_pwm_slice,1);
    pwm_set_enabled(right_side_motor_pwm_slice,1);
    pwm_set_enabled(arm_extension_pwm_slice,1);
    pwm_set_enabled(arm_rotation_pwm_slice,1);
    

    Mechiane.init();

    repeating_timer timer;
    add_repeating_timer_us(5, repeating_timer_callback, NULL, &timer);


    multicore_launch_core1(second_core_test);
    while (true) 
    {
       if(to_ms_since_boot(get_absolute_time())-coms_time_stamp>=secure_action_timeout)
       {
        pico_frame.motor_bias=0.0;
        pico_frame.motor_power=0.0;
        pico_frame.arm_extension_speed=0.0;
        pico_frame.arm_rotation_speed=0.0;
        speed=0;
       }
    }
}
 