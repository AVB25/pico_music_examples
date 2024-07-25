# include "..\simple_wavetable_synth.hpp"

// Run the code that outputs sound repeatedly to measure how long it takes
// to run.

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

    uint i = 0;
    uint idx_update_pwm = 0;
    uint max_pwm_idx = 100;
    uint32_t next_step_increase;

    uint32_t t_end;
    for(int idx=0; idx < 5; idx++){
        printf("%i\n", 5-idx);
        sleep_ms(1000);
    }
    printf("Running\n");

    // absolute_time_t t_start = get_absolute_time();
    uint32_t t_start = time_us_32();
    while(i<1000000){
        i++;
        produce_next_sample();
    };
    t_end = time_us_32();
    uint32_t t_diff = t_end - t_start;

    printf("Start time: %d us\n", t_start);
    printf("End time: %d us\n", t_end);
    printf("Time to execute 1_000_000 samples: %d us.\n", t_diff);
    printf("Time to execute 1 sample: %f us\n", ((float) t_diff) / 1e6);
}