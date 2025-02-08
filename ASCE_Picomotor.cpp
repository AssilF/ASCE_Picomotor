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


#define secure_action_timeout 800

double speed;
int left_top_motor_speed;
int left_bot_motor_speed;
int right_top_motor_speed;
int right_bot_motor_speed;

double wrap_set[8]; //each slice has its own wrap

control_frame pico_frame;

char frame_buffer[sizeof(pico_frame)];

/*flags,LTmot,LBmot,RTmot,RBmot,Intrp,ArmRotation,ArmPose,ElbowPose,Extension,Pitch,Yaw,Grip*/

char callback[] = "Flushed!";
uint32_t coms_time_stamp;
int fetch_index=0;
void answer()
{
    printf("Sending Answer:");
    fetch_index=0;
    for(int i; i<sizeof(callback);i++)
    {
    Wire.write((uint8_t)callback[i]);
    printf("%c",callback[i]);
    }
    printf("\n");
}

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
        this->pulse_width=mapRange(0,this->max_angle,this->minimum,this->maximum,angle);
    }
    void compare_instance(int instance_of_time) // i know bad optimization but I am working without a plan and out of despair bruh
    {
        instance_of_time>this->pulse_width || instance_of_time<this->minimum? gpio_put(this->pin,0):gpio_put(this->pin,1);
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
        this->arm.init(arm_servo_pin,420,2350);
        this->elbow.init(elbow_servo_pin,500,2350);
        this->pitch.init(pitch_servo_pin,600,2200);
        this->yaw.init(yaw_servo_pin,1000,2000);
        this->grip.init(grip_servo_pin,400,2400);
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

bool servo_core(repeating_timer *t) {
    pwm_reference_count+=5;    
    if(pwm_reference_count>20000){pwm_reference_count=0;}//cap em frequencies at 20ms

    Mechiane.run_engine(pwm_reference_count);    

    return true;
}



void drive_motors() {
    // Retrieve and constrain the overall speed (percentage)
    float sp = constrain(pico_frame.motor_power, -1.0f, 1.0f);
    // Read the raw bias (expected range -1 to 1)
    float bias = pico_frame.motor_bias;

    // Determine drive mode: if bit0 of flag_set is set, use rotation mode.
    bool rotation_mode = (pico_frame.flag_set & 0x01);
    float left_speed, right_speed;
    
    if (rotation_mode) { //optimize here, this is for the bias deadzone and not for rotiation mode, it may not be necessary since the bias information aside from the sign, isn't necessary.
    if (fabsf(bias) < JOYSTICK_DEADZONE)
        bias = 0.0f;
    else
        bias = (bias > 0) ? (bias - JOYSTICK_DEADZONE) / (1.0f - JOYSTICK_DEADZONE)
                          : (bias + JOYSTICK_DEADZONE) / (1.0f - JOYSTICK_DEADZONE);
    }
    
    
if (rotation_mode) { //pivot mode
    if (bias > 0) {  
        left_speed  = sp;  
        right_speed = -sp; 
    } else if (bias < 0) {  
        left_speed  = -sp;  
        right_speed = sp;  
    } else {  
        left_speed  = sp;  
        right_speed = sp;  
    }
} else {  
    // Bias mode: Differential steering (one side moves faster)
    left_speed  = sp * (1.0f - bias);
    right_speed = sp * (1.0f + bias);
}


    // Compute absolute speeds (for PWM duty calculations)
    float left_duty  = fabsf(left_speed);
    float right_duty = fabsf(right_speed);
    
    // Quadratic mapping: target frequency scales with (duty/100)^2.
    uint32_t left_target_freq  = PWM_FREQ_MIN + (uint32_t)((PWM_FREQ_MAX - PWM_FREQ_MIN) * (left_duty) * (left_duty));
    uint32_t right_target_freq = PWM_FREQ_MIN + (uint32_t)((PWM_FREQ_MAX - PWM_FREQ_MIN) * (right_duty) * (right_duty));
    
    // Calculate PWM configuration for left and right sides.
    pwm_divisor left_pwm, right_pwm;
    calculate_PWM_div(&left_pwm, left_target_freq);
    calculate_PWM_div(&right_pwm, right_target_freq);
    
    // Apply the calculated clock divider and wrap to the PWM slices.
    pwm_set_clkdiv(left_side_motor_pwm_slice, left_pwm.clk_div);
    pwm_set_wrap(left_side_motor_pwm_slice, left_pwm.wrap);
    pwm_set_clkdiv(right_side_motor_pwm_slice, right_pwm.clk_div);
    pwm_set_wrap(right_side_motor_pwm_slice, right_pwm.wrap);
    
    // Retrieve the wrap values directly.
    uint16_t left_wrap  = get_pwm_wrap(left_side_motor_pwm_slice);
    uint16_t right_wrap = get_pwm_wrap(right_side_motor_pwm_slice);
    
    // Set the PWM duty cycle levels (always using a positive value).
    uint32_t left_level  = (uint32_t)(left_duty  * left_wrap);
    uint32_t right_level = (uint32_t)(right_duty * right_wrap);
    pwm_set_gpio_level(left_top_motor_PWM_pin, left_level);
    pwm_set_gpio_level(left_bot_motor_PWM_pin, left_level);
    pwm_set_gpio_level(right_top_motor_PWM_pin, right_level);
    pwm_set_gpio_level(right_bot_motor_PWM_pin, right_level);
    
    // Set motor direction pins based solely on the sign of the computed speeds.
    bool left_forward  = (left_speed  > 0);
    bool right_forward = (right_speed > 0);
    gpio_put(left_top_motor_p_pin, left_forward);
    gpio_put(left_top_motor_n_pin, !left_forward);
    gpio_put(left_bot_motor_p_pin, left_forward);
    gpio_put(left_bot_motor_n_pin, !left_forward);
    gpio_put(right_top_motor_p_pin, right_forward);
    gpio_put(right_top_motor_n_pin, !right_forward);
    gpio_put(right_bot_motor_p_pin, right_forward);
    gpio_put(right_bot_motor_n_pin, !right_forward);
}




void second_core_test() //behold the hardcoding catastrophe, I won't otpimize it because I am melting and I am literally no-braining this thing
{
    drive_motors();
    sleep_ms(1);
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
    add_repeating_timer_us(5, servo_core, NULL, &timer);

    uint32_t debug_instance;

    multicore_launch_core1(second_core_test);
    while (true) 
    {
        
       if(to_ms_since_boot(get_absolute_time())-coms_time_stamp>=secure_action_timeout)
       {
        fetch_index=0;
        pico_frame.motor_bias=0.0;
        pico_frame.motor_power=0.0;
        pico_frame.arm_extension_speed=0.0;
        pico_frame.arm_rotation_speed=0.0;
        speed=0;
       }

        //Mechiane controls unpacking (Maybe implement easing?)
        Mechiane.arm.setPose(180-pico_frame.arm_servo_pose);
        Mechiane.elbow.setPose(180-pico_frame.elbow_servo_pose);
        Mechiane.pitch.setPose(180-pico_frame.pitch_servo_pose);
        Mechiane.yaw.setPose(pico_frame.yaw_servo_pose);
        Mechiane.grip.setPose(pico_frame.grip_servo_pose);


        if(pico_frame.arm_rotation_speed<0.0)
        {
            pwm_set_gpio_level(arm_rotation_motor_n_pin,0);
            pwm_set_gpio_level(arm_rotation_motor_p_pin,(int)(pico_frame.arm_rotation_speed*wrap_set[arm_rotation_pwm_slice]));
        }else
        {            pwm_set_gpio_level(arm_rotation_motor_n_pin,(int)(-pico_frame.arm_rotation_speed*wrap_set[arm_rotation_pwm_slice]));
            pwm_set_gpio_level(arm_rotation_motor_p_pin,0);

        }
        // sleep_ms(1);
        if(pico_frame.arm_extension_speed<0.0)
        {
            pwm_set_gpio_level(arm_extension_motor_n_pin,(int)(-pico_frame.arm_extension_speed*wrap_set[arm_extension_pwm_slice]));
            pwm_set_gpio_level(arm_extension_motor_p_pin,0);
        }else
        {
            pwm_set_gpio_level(arm_extension_motor_n_pin,0);
            pwm_set_gpio_level(arm_extension_motor_p_pin,(int)(pico_frame.arm_extension_speed*wrap_set[arm_extension_pwm_slice]));
        }
        sleep_ms(1);
    //    if(to_ms_since_boot(get_absolute_time())-debug_instance>=400){
    //    printf("Security Timestamp:%i\n",to_ms_since_boot(get_absolute_time())-coms_time_stamp);
    //    unpack_frame();
    //    debug_instance=to_ms_since_boot(get_absolute_time());
    //    printf("Speed is about:%f\nWrap about at:%f\n",speed,wrap_set[left_side_motor_pwm_slice]);
    //    }

    }
}
 