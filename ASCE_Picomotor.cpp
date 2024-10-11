#include <stdio.h>
#include "pico/stdlib.h"
#include <include/ASCE_Picomotor.h>
#include "pico/i2c_slave.h"
#include "Wire.h"
#include "inttypes.h"
#include "string.h"
#include "hardware/pwm.h"

int left_top_motor_speed;
int left_bot_motor_speed;
int right_top_motor_speed;
int right_bot_motor_speed;

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
    //printf("\n");
}

void unpack_frame()
{ //add further treatment like mapping signs and such
  printf("packed data into corresponding variables wherease: flagset=%X\nLTmot=%i\nLBmot=%i\nRTmot=%i\nRBmot=%i\nArm rotation=%i\nArm pose=%i\nElbow pose=%i\nExtension speed=%i\nPitch=%i\nYaw=%i\nGrip=%i\n",pico_frame.flag_set, pico_frame.left_top_motor_target, pico_frame.left_bot_motor_target, pico_frame.right_top_motor_target, pico_frame.right_bot_motor_target, pico_frame.arm_rotation_speed, pico_frame.arm_servo_pose, pico_frame.elbow_servo_pose, pico_frame.arm_extension_speed, pico_frame.pitch_servo_pose, pico_frame.yaw_servo_pose, pico_frame.grip_servo_pose);
  printf("Ease:%f\n",pico_frame.ease_value);
}

int main()
{   
    stdio_init_all();
    
    LT_mot_pwm_config=pwm_get_default_config();
    LB_mot_pwm_config=pwm_get_default_config();
    RT_mot_pwm_config=pwm_get_default_config();
    RB_mot_pwm_config=pwm_get_default_config();
    arm_extension_p_pwm_config=pwm_get_default_config();
    arm_extension_n_pwm_config=pwm_get_default_config();
    arm_servo_pwm_config=pwm_get_default_config();
    elbow_servo_pwm_config=pwm_get_default_config();
    pitch_servo_pwm_config=pwm_get_default_config();
    yaw_servo_pwm_config=pwm_get_default_config(); 

    //lez start configuring this stuff *sigh* (first commit commence here)

    gpio_init(25);
    gpio_set_dir(25,1);

    gpio_init(left_top_motor_n_pin);
    gpio_init(left_top_motor_p_pin);
    gpio_set_function(left_top_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_init(right_top_motor_n_pin);
    gpio_init(right_top_motor_p_pin);
    gpio_set_function(right_top_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_init(left_bot_motor_n_pin);
    gpio_init(left_bot_motor_p_pin);
    gpio_set_function(left_bot_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_init(right_bot_motor_n_pin);
    gpio_init(right_bot_motor_p_pin);
    gpio_set_function(right_bot_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_set_function(arm_rotation_motor_n_pin,GPIO_FUNC_PWM);
    gpio_set_function(arm_rotation_motor_p_pin,GPIO_FUNC_PWM);
    gpio_set_function(arm_servo_pin,GPIO_FUNC_PWM);
    gpio_set_function(elbow_servo_pin,GPIO_FUNC_PWM);
    gpio_set_function(arm_extension_motor_n_pin,GPIO_FUNC_PWM);
    gpio_set_function(arm_extension_motor_p_pin,GPIO_FUNC_PWM);
    gpio_set_function(pitch_servo_pin,GPIO_FUNC_PWM);
    gpio_set_function(yaw_servo_pin,GPIO_FUNC_PWM);
    gpio_set_function(grip_servo_pin,GPIO_FUNC_PWM);

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
    
    while (true) 
    {
        sleep_ms(50);
        unpack_frame();
    }
}
