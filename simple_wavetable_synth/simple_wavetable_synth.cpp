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
    CurrentStatus current_status = {
        0,  // current_step
        0,  // current_output_sample
        1   // volume
    };
    
    // Initialise the object that stores the sinusoidal waveform
    Waveform waveform = {};
    for (int i = 0; i < N_SAMPLES_STORED; i++){
        waveform.samples[i] = round((PWM_HALF_BIT_DEPTH - 1) * sin(2 * M_PI * i / N_SAMPLES_STORED));
    }

    // Initialise the UserData struct that is passed to the repeating timer and stores the
    // waveform and the current status
    UserData user_data = {
        &waveform,
        &current_status
    };

    // Initialise the repeating timer and set going
    repeating_timer_t rt;
    add_repeating_timer_us(iSamplePeriod_us, produce_next_sample, &user_data, &rt);
    while(1){};
}