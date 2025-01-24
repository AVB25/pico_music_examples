#ifndef _PICO_HW_UTIL_PWM
#define _PICO_HW_UTIL_PWM

/**
 * \brief Initialise PWM.
 * 
 * \param pwm_slice The PWM slice to initialise
 * \param pwm_gpio  The GPIO pin used for the PWM
 * \param bit_depth The desired PWM bit depth (sets the PWM wrap value)
 */
void init_pwm(uint pwm_slice, uint pwm_gpio, uint bit_depth);

#endif