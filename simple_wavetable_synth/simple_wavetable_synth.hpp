# include <stdio.h>
# include <math.h>
# include "pico/stdio.h"
# include "hardware/adc.h"
# include "pico/stdlib.h"
# include "hardware/interp.h"
# include "hardware/pwm.h"


// Sample rate
# define F_SAMPLE 44000
const int32_t iSamplePeriod_us = 1e6 / F_SAMPLE;

// Only sample the ADC every 100 periods
# define ADC_SAMPLE_PERIOD 100
uint current_adc_sample = 0;


/**
 * Define sample depth of the waveform. One waveform period is sampled with 8
 * bit depth (256 samples) and the interpolator gives an effective extra 8 bits
 * of sampling, giving an effective 16 bit sample depth. So the period in steps will be
 * a 16 bit word, whose first half gives the index in the wavetable that the sample
 * roughly corresponds to, and the second is the interpolation fraction between it
 * and the next sample.
 */
# define N_SAMPLES_TOT 65536    // 2^16
# define N_SAMPLES_STORED 256   // 2^8
# define N_SAMPLES_INTERP 256   // 2^8


/**
 * \brief Waveform struct that contains the samples of the waveform and the current step to output. 
 * 
 * \param samples Array storing the samples. They are stored as 32 bit ints, since the base[]
 * values for the interpolators (between which the interpolation happens) are 32 bits. Only 
 * the last 8 bits are non-zero  (set by PWM_BIT_DEPTH).
*/
typedef struct {
    int32_t samples[N_SAMPLES_STORED];
} Waveform;
Waveform waveform;


/**
 * \brief Struct that contains the current state of the synthesiser. 
 * 
 * \param current_step 16 bit word containing the current step along the waveform the last outpu
 * was at. MSB 8 bits are the "rough" sample, which is the index along the waveform to take as the 
 * lowest end in the interpolation with the next entry in the waveform. LSB 8 bits are the "fine"
 * sample, which is the amount of interpolation between them. 
 * \param current_output_sample Sample to output
 * \param volume The volume of the synth. Float between 0 and 1.
 */
typedef struct {
    int32_t current_step;
    int32_t current_output_sample;
    float volume;
} CurrentStatus;
CurrentStatus current_status;


/**
 * The sample rate and sample depth define the maximum frequency resolution as
 * df_min = f_s / N. But I will determine the frequency from the ADC reading 
 * for now, which has a resolution of 12 bits. This sets the effective sample depth
 * in the waveform. 
 * In any case, I will set a minimum frequency of 10 Hz and a maximum of 22 kHz. This
 * determines the maximum and minimum number of steps per sample period. The interpolator
 * is 8 bits, but the extremes (the base[] values) are 32 bits. However, only the last 16 
 * bits are non-zero (since N_SAMPLES_TOT is 16 bits)
 */
# define F_MIN 10
# define F_MAX 22000

const uint32_t iMinNSteps =  (F_MIN * N_SAMPLES_TOT) / F_SAMPLE;
const uint32_t iMaxNSteps =  (F_MAX * N_SAMPLES_TOT) / F_SAMPLE;


/**
 * The interpolator is 8 bits. To make full use of the 12 bits of the ADC when 
 * interpolating between the min and max number of steps, I will interpolate
 * twice. One will be an 8 bit interpolation between iMinNSteps and iMaxNSteps,
 * and the next will be a 4 bit interpolation between the previous result and it plus
 * 8 bit resolution between iMinNSteps and iMaxNSteps. 32 bits since it will be the
 * extreme of an interpolation
 */

const uint32_t iNSteps8Bit = (iMaxNSteps - iMinNSteps) / 256;


// Use PWM slice 0, channel A, which is GPIO pin 0 (pin 1 on the board :P)
# define PWM_SLICE 0
# define PWM_CHAN PWM_CHAN_A
# define PWM_GPIO 0
# define PWM_BIT_DEPTH 256
# define PWM_HALF_BIT_DEPTH 128


/**
 * \brief Initialise PWM.
 * 
 * \param pwm_slice The PWM slice to initialise
 * \param bit_depth The desired PWM bit depth (sets the PWM wrap value)
 */
void init_pwm(uint pwm_slice, uint bit_depth){
    gpio_set_function(0, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_slice, bit_depth - 1);
    pwm_set_enabled(pwm_slice, true);
}


// Use the ADC on GPIO 26
# define PIN_ADC 26
// Use ADC input 0
# define ADC_INPUT 0


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

/**
 * Initialise interpolator interp0 to blend mode.
 */
void initialise_interpolator(){
    // Initialise dynamic interpolator
    interp_config cfg = interp_default_config();
    // Lane 0
    interp_config_set_blend(&cfg, true);
    interp_set_config(interp0, 0, &cfg);
    // Lane 1
    interp_config dflt_cfg = interp_default_config();
    interp_set_config(interp0, 1, &dflt_cfg);
}


/**
 * Use the hardware interpolator to interpolate between two positive values.
 * 
 * \param low_level Low end of interpolation
 * \param high_level High end of interpolation
 * \param interp_fraction Interpolation fraction. Only last 8 bits are used.
 */
inline uint32_t _interpolate_unsigned(
    uint32_t low_level,
    uint32_t high_level,
    uint32_t interp_fraction
    ){
        // Set interpolator to unsigned
        hw_write_masked(&(interp0->ctrl[1]), 0 << 15, 1 << 15);
        interp0->base[0] = low_level;
        interp0->base[1] = high_level;
        // Keep last 4 bits and shift left to make them 8-bit MSBs
        interp0->accum[1] = interp_fraction;

        return interp0->peek[1];
}


/**
 * Use the hardware interpolator to interpolate between two signed values.
 * 
 * \param low_level Low end of interpolation
 * \param high_level High end of interpolation
 * \param interp_fraction Interpolation fraction. Only last 8 bits are used.
 */
inline int32_t _interpolate_signed(
    int32_t low_level,
    int32_t high_level,
    int32_t interp_fraction
    ){
        // Set interpolator to signed
        hw_write_masked(&(interp0->ctrl[1]), 1 << 15, 1 << 15);
        interp0->base[0] = low_level;
        interp0->base[1] = high_level;
        // Keep last 4 bits and shift left to make them 8-bit MSBs
        interp0->accum[1] = interp_fraction;

        return interp0->peek[1];
}


/**
 * \brief Return the number of steps by which to advance the waveform for outputting
 * the next sample. 
 * 
 * \param adc_freq_reading 12 bit ADC reading, which linearly sets the frequency.
 * \return 32 bit int containing the 16 bit step size for the next output as its last 16 
 *      bits 
 */
uint32_t get_next_step_increase(
    uint16_t adc_freq_reading
    ){
        // Get intermediate 8-bit interpolation
        uint32_t intResult = _interpolate_unsigned(
            iMinNSteps,
            iMaxNSteps,
            (uint32_t) adc_freq_reading >> 4
        );


        // Get remaining 4-bits of interpolation
        uint32_t next_step_size = _interpolate_unsigned(
            intResult,
            intResult + iNSteps8Bit,
            (uint32_t) (adc_freq_reading & 0xf) << 4 // Take remaining 4 bits and make 8 bit long
            );
        return next_step_size & 0xffff;    // Cast to 16 bits, since that's the full, maximum bit depth
}


/**
 * \brief Return the next sample to output to the PWM, which is taken as an interpolation between
 * two consecutive waveform samples. The sample to pick and the interpolation fraction are
 * given by the step size. Also update the current step.
 * 
 * \param step_increase Number of steps to advance in the next sample, split into two 8 bit halves. The first half
 * is the index within the waveform to read, the second is the amount of interpolation to do between
 * that and the next waveform sample.
 * \param status CurrentStatus object pointer that contains the status of the synthesiser. Is mutated by the function.
 * \param waveform The waveform from which to output.
 * 
 */
void update_state_params(
    uint32_t step_increase,
    CurrentStatus* status,
    Waveform* waveform
    ){
        int32_t next_step = (status->current_step + step_increase) % N_SAMPLES_TOT;
        int32_t* waveform_sample_low = waveform->samples + (next_step >> 8);    // Index of the waveform to take as the low interpolation extreme
        int32_t* waveform_sample_high = waveform->samples + ((next_step >> 8) + 1) % N_SAMPLES_STORED;    // Index of the waveform to take as the high interpolation extreme
        int32_t interpolation_word = next_step & 0xff;  // Amount of interpolation

        int32_t _current_output_sample = _interpolate_signed(
            *waveform_sample_low,
            *waveform_sample_high,
            interpolation_word
        );

        status->current_step = next_step; // Update waveform current step 
        status->current_output_sample = (status->volume * _current_output_sample) + PWM_HALF_BIT_DEPTH;
}


// Store the next ADC reading
uint16_t adc_freq_reading;
// Store the next step increase
uint32_t next_step_increase = 0;


/**
 * \brief Function that calculates and produces the next sample. Is implemented as the callback of
 * a repeating alarm.
 * 
 * \param rt Repeating user timer that contains the pointer to the state parameters (stored in 
 * the user_data).
 */
bool produce_next_sample(repeating_timer_t* rt){
    if(current_adc_sample < ADC_SAMPLE_PERIOD){
        current_adc_sample++;
    }
    else{
        adc_freq_reading = adc_read();
        next_step_increase = get_next_step_increase(adc_freq_reading);
        current_adc_sample = 0;
    }
    update_state_params(
        next_step_increase,
        &current_status,
        &waveform
    );
    pwm_set_chan_level(PWM_SLICE, PWM_CHAN, current_status.current_output_sample);
    return true;
}


