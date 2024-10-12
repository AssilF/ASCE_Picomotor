#include <stdio.h>
#include "pico/stdlib.h"
#include <include/ASCE_Picomotor.h>
#include "pico/i2c_slave.h"
#include "Wire.h"
#include "inttypes.h"
#include "string.h"
#include "hardware/pwm.h"
#include <math.h>

int left_top_motor_speed;
int left_bot_motor_speed;
int right_top_motor_speed;
int right_bot_motor_speed;

uint16_t wrap_set[8]; //each slice has its own wrap

void position_servo(int servo_pin ,float angle)
{
    int current_wrap=wrap_set[pwm_gpio_to_slice_num(servo_pin)];
    pwm_set_gpio_level(servo_pin,(int)mapRange(1,180,current_wrap*0.05,current_wrap*0.1,angle));
}

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

    left_side_motor_pwm_config=pwm_get_default_config();
    right_side_motor_pwm_config=pwm_get_default_config();
    arm_extension_pwm_config=pwm_get_default_config();
    arm_rotation_pwm_config=pwm_get_default_config();
    arm_elbow_servo_pwm_config=pwm_get_default_config();
    pitch_yaw_servo_pwm_config=pwm_get_default_config();
    grip_servo_pwm_config=pwm_get_default_config(); 

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

//configuring the servos
    calculate_PWM_div(&div,MG996r_ferq);
    pwm_config_set_clkdiv(&arm_elbow_servo_pwm_config,64.f);
    pwm_config_set_wrap(&arm_elbow_servo_pwm_config,3906.2f); 
    wrap_set[arm_elbow_servo_pwm_slice]=3906.2f;
    printf("MG996R div: %f and wrap: %i\n",div.clk_div,div.wrap);



    calculate_PWM_div(&div,SG90_freq);
    pwm_config_set_clkdiv(&pitch_yaw_servo_pwm_config,div.clk_div);
    pwm_config_set_clkdiv(&grip_servo_pwm_config,div.clk_div); //I know what you think about this, why everyone got its own variable, answer is, idk man I may want to tweak things seperately, but virtually we only needed 2 profiles or configs
    pwm_config_set_wrap(&pitch_yaw_servo_pwm_config,div.wrap);
    pwm_config_set_wrap(&grip_servo_pwm_config,div.wrap);
    wrap_set[pitch_yaw_servo_pwm_slice]=div.wrap;
    wrap_set[grip_servo_pwm_slice]=div.wrap;
    printf("SG90 div: %f and wrap: %i\n",div.clk_div,div.wrap);

    //again, none of this is necessary we could have just hardcoded this but my adhd brain is just not letting me be :(


    pwm_init(left_side_motor_pwm_slice,&left_side_motor_pwm_config,0);
    pwm_init(right_side_motor_pwm_slice,&right_side_motor_pwm_config,0);
    pwm_init(arm_extension_pwm_slice,&arm_extension_pwm_config,0);
    pwm_init(arm_rotation_pwm_slice,&arm_rotation_pwm_config,0);
    pwm_init(pitch_yaw_servo_pwm_slice,&pitch_yaw_servo_pwm_config,0);
    pwm_init(grip_servo_pwm_slice,&grip_servo_pwm_config,0);

    gpio_init(25);
    gpio_set_dir(25,1);

    gpio_init(left_top_motor_n_pin);
    gpio_init(left_top_motor_p_pin);
    gpio_init(left_top_motor_PWM_pin);
    gpio_set_function(left_top_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_init(right_top_motor_n_pin);
    gpio_init(right_top_motor_p_pin);
    gpio_init(right_top_motor_PWM_pin);
    gpio_set_function(right_top_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_init(left_bot_motor_n_pin);
    gpio_init(left_bot_motor_p_pin);
    gpio_init(left_bot_motor_PWM_pin);
    gpio_set_function(left_bot_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_init(right_bot_motor_n_pin);
    gpio_init(right_bot_motor_p_pin);
    gpio_init(right_bot_motor_PWM_pin);
    gpio_set_function(right_bot_motor_PWM_pin,GPIO_FUNC_PWM);

    gpio_init(arm_rotation_motor_n_pin);
    gpio_init(arm_rotation_motor_p_pin);
    gpio_init(arm_servo_pin);
    gpio_init(elbow_servo_pin);
    gpio_init(arm_extension_motor_n_pin);
    gpio_init(arm_extension_motor_p_pin);
    gpio_init(pitch_servo_pin);
    gpio_init(yaw_servo_pin);
    gpio_init(grip_servo_pin);

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
    
    pwm_set_enabled(left_side_motor_pwm_slice,1);
    pwm_set_enabled(right_side_motor_pwm_slice,1);
    pwm_set_enabled(arm_extension_pwm_slice,1);
    pwm_set_enabled(arm_rotation_pwm_slice,1);
    pwm_set_enabled(arm_elbow_servo_pwm_slice,1);
    pwm_set_enabled(pitch_yaw_servo_pwm_slice,1);
    pwm_set_enabled(grip_servo_pwm_slice,1);

    while (true) 
    {
        for(int a=1;a<180;a++){
        position_servo(arm_servo_pin,a);
        sleep_ms(20);
        }
        for(int a=180;a>1;a--){
        position_servo(arm_servo_pin,a);
        sleep_ms(20);
        }
        unpack_frame();
    }
}
