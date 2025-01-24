#ifndef _HARDWARE_PWM_H
    # include "hardware/pwm.h"
#endif
#ifndef _PICO_STDLIB_H
    # include "pico/stdlib.h"
#endif
# include "pwm.hpp" 

void init_pwm(uint pwm_slice, uint pwm_gpio, uint bit_depth){
    gpio_set_function(pwm_gpio, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_slice, bit_depth - 1);
    pwm_set_enabled(pwm_slice, true);
}