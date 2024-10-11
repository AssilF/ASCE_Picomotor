#include <stdio.h>
#include "pico/stdlib.h"
#include <include/ASCE_Picomotor.h>
#include "pico/i2c_slave.h"
#include "Wire.h"
#include "inttypes.h"
#include "string.h"
#include "hardware/pwm.h"
#include <math.h>

#define system_freq 125e6
#define target_motor_freq 80e3
#define motor_PWM_Freq_div ((system_freq)/(target_motor_freq))
#define MG996r_ferq 50 //50-60 ig ?
#define SG90_freq 50
#define MG996r_PWM_Freq_div ((system_freq)/(MG996r_ferq))
#define SG90_PWM_Freq_div ((system_freq)/(SG90_freq))
#define us_to_pulse_count(us,freq) ((us*65536)/((1e6/freq)))

#define MG996r_maximum_pulse_width 2000 //in µs
#define MG996r_minimum_pulse_width 1000

#define SG90_maximum_pulse_width 2000
#define SG90_minimum_pulse_width 1000

#define servo_angle(degrees,min_pulse,max_pulse,freq,max_angle) (mapRange(0,max_angle,us_to_pulse_count(min_pulse,freq),us_to_pulse_count(max_pulse,freq),degrees))

int mapRange(int a1,int a2,int b1,int b2,int s) {
  return b1 + (s-a1)*(b2-b1)/(a2-a1);
}

int constrain(int x, int a, int b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

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
    
    left_side_motor_pwm_config=pwm_get_default_config();
    right_side_motor_pwm_config=pwm_get_default_config();
    arm_extension_pwm_config=pwm_get_default_config();
    arm_rotation_pwm_config=pwm_get_default_config();
    arm_elbow_servo_pwm_config=pwm_get_default_config();
    pitch_yaw_servo_pwm_config=pwm_get_default_config();
    grip_servo_pwm_config=pwm_get_default_config(); 

    //lez start configuring this stuff *sigh* (first commit commence here) //default PWM clock speed is about 125MHz so do your math accordingly (I am talking to you my future self)
    pwm_config_set_clkdiv(&left_side_motor_pwm_config ,motor_PWM_Freq_div); //this gives us a frequency of 80KHz (aka one wrap each 0.0008). . . but like, hold up, when will the pwm register wrap ? (I beleive the counter buffer is like, 16 bits ? about up to 65,536)
    pwm_config_set_clkdiv(&right_side_motor_pwm_config,motor_PWM_Freq_div);
    pwm_config_set_clkdiv(&arm_extension_pwm_config,motor_PWM_Freq_div);
    pwm_config_set_clkdiv(&arm_rotation_pwm_config,motor_PWM_Freq_div);
    pwm_config_set_clkdiv(&arm_elbow_servo_pwm_config,MG996r_PWM_Freq_div);
    pwm_config_set_clkdiv(&pitch_yaw_servo_pwm_config,SG90_PWM_Freq_div);
    pwm_config_set_clkdiv(&grip_servo_pwm_config,SG90_PWM_Freq_div); //I know what you think about this, why everyone got its own variable, answer is, idk man I may want to tweak things seperately, but virtually we only needed 2 profiles or configs

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
    
    pwm_set_enabled(left_side_motor_pwm_slice,1);
    pwm_set_enabled(left_side_motor_pwm_slice,1);
    pwm_set_enabled(arm_elbow_servo_pwm_slice,1);

    while (true) 
    {
        sleep_us(50);
        static int i;
        static int t;


        for(int a=0; a<=180; a++){
        pwm_set_gpio_level(arm_servo_pin,1639);
        i++;
        t+=10;
        sleep_ms(50);
        pwm_set_both_levels(left_side_motor_pwm_slice,i,t);
        }
        for(int a=180; a>0; a--){
        pwm_set_gpio_level(arm_servo_pin,);
        sleep_ms(50);
                i++;
        t+=10;
                pwm_set_both_levels(left_side_motor_pwm_slice,i,t);
        }


        unpack_frame();
    }
}
