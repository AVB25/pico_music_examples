#ifndef _PICO_HW_UTIL_ADC
#define _PICO_HW_UTIL_ADC
/** 
 * \brief Initialise the ADC
 * 
 * \param pin_adc Pin on which to sample the ADC.
 * \param adc_intpu ADC input to select
 */
void init_adc(uint pin_adc, uint adc_input);

#endif