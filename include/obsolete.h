


// while(1){
//      if(pico_frame.flag_set==1){
//             if(pico_frame.motor_power>=0.0)
//             {
//                 gpio_put(left_top_motor_n_pin,1);
//                 gpio_put(left_top_motor_p_pin,0);
//                 gpio_put(right_top_motor_n_pin,1);
//                 gpio_put(right_top_motor_p_pin,0);
//                 gpio_put(left_bot_motor_n_pin,1);
//                 gpio_put(left_bot_motor_p_pin,0);
//                 gpio_put(right_bot_motor_n_pin,1);
//                 gpio_put(right_bot_motor_p_pin,0);
//                 speed = (pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
//                 if(pico_frame.motor_bias>0.2){
//                                     gpio_put(left_bot_motor_n_pin,!gpio_get_out_level(left_bot_motor_n_pin));
//                 gpio_put(left_bot_motor_p_pin,!gpio_get_out_level(left_bot_motor_p_pin));
//                 gpio_put(left_top_motor_n_pin,!gpio_get_out_level(left_top_motor_n_pin));
//                 gpio_put(left_top_motor_p_pin,!gpio_get_out_level(left_top_motor_p_pin));

//                 pwm_set_gpio_level(left_top_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_top_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 }else if(pico_frame.motor_bias<-0.2)
//                 {
//                 gpio_put(right_bot_motor_n_pin,!gpio_get_out_level(right_bot_motor_n_pin));
//                 gpio_put(right_bot_motor_p_pin,!gpio_get_out_level(right_bot_motor_p_pin));
//                 gpio_put(right_top_motor_n_pin,!gpio_get_out_level(right_top_motor_n_pin));
//                 gpio_put(right_top_motor_p_pin,!gpio_get_out_level(right_top_motor_p_pin));
//                 pwm_set_gpio_level(left_top_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_top_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 }else
//                 {
//                 pwm_set_gpio_level(left_top_motor_PWM_pin,(int)speed);
//                 pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)speed);
//                 pwm_set_gpio_level(right_top_motor_PWM_pin,(int)speed);
//                 pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)speed);
//                 }
//             }else
//             {
//                 gpio_put(left_top_motor_n_pin,0);
//                 gpio_put(left_top_motor_p_pin,1); 
//                 gpio_put(right_top_motor_n_pin,0);
//                 gpio_put(right_top_motor_p_pin,1);
//                 gpio_put(left_bot_motor_n_pin,0);
//                 gpio_put(left_bot_motor_p_pin,1);
//                 gpio_put(right_bot_motor_n_pin,0);
//                 gpio_put(right_bot_motor_p_pin,1);
//                 speed = (-pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
//                 if(pico_frame.motor_bias>0.2){
//                 gpio_put(left_bot_motor_n_pin,!gpio_get_out_level(left_bot_motor_n_pin));
//                 gpio_put(left_bot_motor_p_pin,!gpio_get_out_level(left_bot_motor_p_pin));
//                 gpio_put(left_top_motor_n_pin,!gpio_get_out_level(left_top_motor_n_pin));
//                 gpio_put(left_top_motor_p_pin,!gpio_get_out_level(left_top_motor_p_pin));
//                 pwm_set_gpio_level(left_top_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_top_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)(pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 }else if(pico_frame.motor_bias<-0.2)
//                 {
//                 gpio_put(right_bot_motor_n_pin,!gpio_get_out_level(right_bot_motor_n_pin));
//                 gpio_put(right_bot_motor_p_pin,!gpio_get_out_level(right_bot_motor_p_pin));
//                 gpio_put(right_top_motor_n_pin,!gpio_get_out_level(right_top_motor_n_pin));
//                 gpio_put(right_top_motor_p_pin,!gpio_get_out_level(right_top_motor_p_pin));
//                 pwm_set_gpio_level(left_top_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_top_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)(-pico_frame.motor_bias*wrap_set[left_side_motor_pwm_slice]));
//                 }else
//                 {
//                 pwm_set_gpio_level(left_top_motor_PWM_pin,(int)speed);
//                 pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)speed);
//                 pwm_set_gpio_level(right_top_motor_PWM_pin,(int)speed);
//                 pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)speed);
//                 }
//         }
//         }else{
//         if(pico_frame.motor_power>=0.0)
        
//         {
//             gpio_put(left_top_motor_n_pin,1);
//             gpio_put(left_top_motor_p_pin,0);
//             gpio_put(right_top_motor_n_pin,1);
//             gpio_put(right_top_motor_p_pin,0);
//             gpio_put(left_bot_motor_n_pin,1);
//             gpio_put(left_bot_motor_p_pin,0);
//             gpio_put(right_bot_motor_n_pin,1);
//             gpio_put(right_bot_motor_p_pin,0);
//             speed = (pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
            
//             if(pico_frame.motor_bias>=0.0){
//             pwm_set_gpio_level(left_top_motor_PWM_pin,(int)speed);
//             pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)speed);
//             pwm_set_gpio_level(right_top_motor_PWM_pin,(int)speed-speed*(pico_frame.motor_bias));
//             pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)speed- speed*(pico_frame.motor_bias));
//             }else
//             {
//             pwm_set_gpio_level(left_top_motor_PWM_pin,(int)speed-speed*(-pico_frame.motor_bias));
//             pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)speed-speed*(-pico_frame.motor_bias));
//             pwm_set_gpio_level(right_top_motor_PWM_pin,(int)speed);
//             pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)speed);
//             }
//         }else
//         {
//             gpio_put(left_top_motor_n_pin,0);
//             gpio_put(left_top_motor_p_pin,1); 
//             gpio_put(right_top_motor_n_pin,0);
//             gpio_put(right_top_motor_p_pin,1);
//             gpio_put(left_bot_motor_n_pin,0);
//             gpio_put(left_bot_motor_p_pin,1);
//             gpio_put(right_bot_motor_n_pin,0);
//             gpio_put(right_bot_motor_p_pin,1);
//             speed = (-pico_frame.motor_power*wrap_set[left_side_motor_pwm_slice]);
//             if(pico_frame.motor_bias>=0.0){
//             pwm_set_gpio_level(left_top_motor_PWM_pin,(int)speed);
//             pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)speed);
//             pwm_set_gpio_level(right_top_motor_PWM_pin,(int)speed-speed*(pico_frame.motor_bias));
//             pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)speed-speed*(pico_frame.motor_bias));
//             }else
//             {
//             pwm_set_gpio_level(left_top_motor_PWM_pin,(int)speed- speed*(-pico_frame.motor_bias));
//             pwm_set_gpio_level(left_bot_motor_PWM_pin,(int)speed- speed*(-pico_frame.motor_bias));
//             pwm_set_gpio_level(right_top_motor_PWM_pin,(int)speed);
//             pwm_set_gpio_level(right_bot_motor_PWM_pin,(int)speed);
//             }
//         }
        
//         }
//         sleep_ms(1);
//         }