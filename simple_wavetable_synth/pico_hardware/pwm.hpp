# include "hardware/pwm.h"
# include "pico/stdlib.h"

/**
 * \brief Initialise PWM.
 * 
 * \param pwm_slice The PWM slice to initialise
 * \param pwm_gpio  The GPIO pin used for the PWM
 * \param bit_depth The desired PWM bit depth (sets the PWM wrap value)
 */
void init_pwm(uint pwm_slice, uint pwm_gpio, uint bit_depth){
    gpio_set_function(pwm_gpio, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_slice, bit_depth - 1);
    pwm_set_enabled(pwm_slice, true);
}