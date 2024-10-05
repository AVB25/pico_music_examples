# include "simple_wavetable_synth.hpp"

int main(){
    stdio_init_all();

    // Sleep necessary to avoid board crashing
    sleep_ms(1);

    // Initialise the PWM, ADC and interpolators
    init_pwm(PWM_SLICE, PWM_BIT_DEPTH);
    init_adc(PIN_ADC, ADC_INPUT);
    initialise_interpolator();

    // Initialise the struct that stores the current status of the synth
    current_status = {
        0,  // current_step
        0,  // current_output_sample
        1<<16 - 1   // volume, max is 65535 (2^16 - 1)
    };

    if(!ensure_valid_current_status(&current_status)){
        return 1;
    }
    
    // Initialise the object that stores the sinusoidal waveform
    waveform = {};
    for (int i = 0; i < N_SAMPLES_STORED; i++){
        waveform.samples[i] = round((PWM_HALF_BIT_DEPTH - 1) * sin(2 * M_PI * i / N_SAMPLES_STORED));
    }

    // Initialise the repeating timer and set going
    repeating_timer_t rt;
    add_repeating_timer_us(-iSamplePeriod_us, callback_produce_next_sample, nullptr, &rt);
    while(1){};
}