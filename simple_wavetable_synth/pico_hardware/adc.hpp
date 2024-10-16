# include "hardware/adc.h"

/**
 * \brief Initialise the ADC
 * 
 * \param pin_adc Pin on which to sample the ADC.
 * \param adc_intpu ADC input to select
 */
void init_adc(uint pin_adc, uint adc_input){
    adc_init();
    adc_gpio_init(pin_adc);
    adc_select_input(adc_input);
}