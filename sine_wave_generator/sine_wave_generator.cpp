# include "pico/stdlib.h"
# include <stdio.h>
# include "hardware/pwm.h"
# include <math.h>

// Use PWM slice 0, channel A, which is GPIO pin 0 (pin 1 on the board :P)
# define PWM_SLICE 0
# define PWM_CHAN PWM_CHAN_A
# define PWM_GPIO 0

// Define frequencies and bit depth of PWM
# define BIT_DEPTH 256
# define F_NOTE 440
# define F_SAMPLE 44000
int64_t iPeriod_us = - round(1e6 / F_SAMPLE);

const int n_samples = F_SAMPLE / F_NOTE;

using namespace std;

uint16_t iWavetable[n_samples]; // Wavetable array that stores the samples in a single sine wave period.
uint16_t iCurrentSampleIdx = 0; // Index of the wavetable containing value to be outpu

// Define UserData struct that contains the pointer to the wavetable
// array and the pointer to the current sample in the array.
struct UserData{
    uint16_t* piWavetable;    // Pointer to the wavetable array
    uint16_t* piCurrentSampleIdx;  // Pointer to the index of the current sample in the array
};

UserData user_data = {
    iWavetable,
    &iCurrentSampleIdx
};

// Define alarm callback function that updates PWM
bool update_pwm(repeating_timer_t* rt){
    UserData* userdata = (UserData*) rt->user_data;
    // Pointer to the sample to be output is the pointer to the start of the wavetable
    // plus the index of the sample to play
    uint16_t* iCurrentSample = userdata->piWavetable + *(userdata->piCurrentSampleIdx);
    pwm_set_chan_level(PWM_SLICE, PWM_CHAN, *iCurrentSample);
    *(userdata->piCurrentSampleIdx) = (*(userdata->piCurrentSampleIdx) + 1) % n_samples;
    return true;  
};

int main(){
    stdio_init_all();
    
    // Without this sleep, stuff doesn't work! I think this is very odd...
    sleep_ms(1);
    
    // Initialise PWM channel
    gpio_set_function(0, GPIO_FUNC_PWM);
    // Set PWM wrap value to the bit depth -1
    pwm_set_wrap(PWM_SLICE, BIT_DEPTH - 1);
    pwm_set_enabled(PWM_SLICE, true);
    // pwm_set_chan_level(PWM_SLICE, PWM_CHAN, round(BIT_DEPTH / 2));

    // Initialise int that stores the note that is passed by user. For now,
    // will be an orangutan and pretend that you can only send 3 digits for the
    // frequency.
    float fNote = 0;

    // Calculate values in wavetable array
    for (int i=0; i<n_samples; i++){
        iWavetable[i] = round((BIT_DEPTH - 1) * 0.5 * (1 + sin(2 * M_PI * F_NOTE / F_SAMPLE * i)));
    };

    // Initialise repeating timer struct
    repeating_timer_t rt;
    add_repeating_timer_us(iPeriod_us, update_pwm, &user_data, &rt);

    // Loop forever
    while(1){
        char nextchar = getchar_timeout_us(0);
        if (nextchar != PICO_ERROR_TIMEOUT){
            int power = 0;
            fNote = 0;
            while(nextchar != PICO_ERROR_TIMEOUT and nextchar != '\n'){
                fNote += ((int) nextchar) * 10^(2 - power);
                power += 1;
                nextchar = getchar_timeout_us(0);
            };
            
        }
    }
}