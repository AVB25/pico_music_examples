# include "../simple_wavetable_synth.hpp"

int main(){
    stdio_init_all();

    // Sleep necessary to avoid board crashing
    sleep_ms(1);

    // Initialise the PWM, ADC and interpolators
    init_pwm(PWM_SLICE, PWM_BIT_DEPTH);
    init_adc(PIN_ADC, ADC_INPUT);
    initialise_interpolator();

    // Initialise the struct that stores the current status of the synth
    CurrentStatus current_status = {
        0,  // current_step
        0,  // current_output_sample
        1.0   // volume
    };
    
    // Initialise the object that stores the sinusoidal waveform
    Waveform waveform = {};
    for (int i = 0; i < N_SAMPLES_STORED; i++){
        waveform.samples[i] = round((PWM_HALF_BIT_DEPTH - 1) * sin(2 * (M_PI / N_SAMPLES_STORED) * i));
    }

    while(1){
        printf("#############################\n");
        printf("min step increase: %d\n", iMinNSteps);
        printf("max step increase: %d\n", iMaxNSteps);
        printf("8-bit step increase: %d\n", iNSteps8Bit);
        uint16_t adc_freq_reading = adc_read();
        printf("adc_reading: %03x\n", adc_freq_reading);
        uint32_t next_step_increase = get_next_step_increase(adc_freq_reading);
        printf("Current step: %x\n", current_status.current_step);
        printf("Next step increase: %x\n", next_step_increase);
        printf("Current step + next_step: %x\n", current_status.current_step + next_step_increase);
        update_state_params(
            next_step_increase,
            &current_status,
            &waveform
        );
        printf("next sample: %i\n", current_status.current_output_sample);
        sleep_ms(500);
    };
}