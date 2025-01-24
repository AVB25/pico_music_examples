#ifndef _HARDWARE_ADC_H
    #include "hardware/adc.h"
#endif
#include "adc.hpp"

void init_adc(uint pin_adc, uint adc_input){
    adc_init();
    adc_gpio_init(pin_adc);
    adc_select_input(adc_input);
}