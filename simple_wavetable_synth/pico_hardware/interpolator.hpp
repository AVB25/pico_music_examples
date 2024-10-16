# include "hardware/interp.h"

/**
 * \brief Initialise interpolator interp0 to blend mode.
 */
void initialise_blend_interpolator(){
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
 * \brief Use the hardware interpolator to interpolate between two positive values.
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
 * \brief Use the hardware interpolator to interpolate between two signed values.
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