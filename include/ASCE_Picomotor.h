#define left_top_motor_p_pin 11 //Each driven @ 14.4 volts 
#define left_top_motor_n_pin 16
#define left_top_motor_PWM_pin 12

#define right_top_motor_p_pin 17
#define right_top_motor_n_pin 18
#define right_top_motor_PWM_pin 13

#define left_bot_motor_p_pin 19
#define left_bot_motor_n_pin 20
#define left_bot_motor_PWM_pin 14

#define right_bot_motor_p_pin 21
#define right_bot_motor_n_pin 22
#define right_bot_motor_PWM_pin 15


#define arm_rotation_motor_p_pin 2  //..
#define arm_rotation_motor_n_pin 3  //..
#define arm_extension_motor_p_pin 4 //..
#define arm_extension_motor_n_pin 5 //All these must be PWM
#define arm_servo_pin 6             //MG996r Check in the datasheet for the servo timings and frequencies 
#define elbow_servo_pin 7           //MG996r
#define pitch_servo_pin 8           //SG90
#define yaw_servo_pin 9             //SG90
#define grip_servo_pin 10        //SG90

pwm_config LT_mot_pwm_config;
pwm_config LB_mot_pwm_config;
pwm_config RT_mot_pwm_config;
pwm_config RB_mot_pwm_config;
pwm_config arm_extension_p_pwm_config;
pwm_config arm_extension_n_pwm_config;
pwm_config arm_rotation_p_pwm_config;
pwm_config arm_rotation_n_pwm_config;
pwm_config arm_servo_pwm_config;
pwm_config elbow_servo_pwm_config;
pwm_config pitch_servo_pwm_config;
pwm_config yaw_servo_pwm_config; //can be upgraded with 4 more servos or motors ig ? but pin configs must change so

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