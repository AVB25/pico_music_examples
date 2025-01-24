#ifndef _PICO_HW_UTIL_INTERP
#define _PICO_HW_UTIL_INTERP

/**
 * \brief Initialise interpolator interp0 to blend mode.
 */
void initialise_blend_interpolator();

/**
 * \brief Use the hardware interpolator to interpolate between two positive values.
 * 
 * \param low_level Low end of interpolation
 * \param high_level High end of interpolation
 * \param interp_fraction Interpolation fraction. Only last 8 bits are used.
 */
uint32_t _interpolate_unsigned(
// inline uint32_t _interpolate_unsigned(
    uint32_t low_level,
    uint32_t high_level,
    uint32_t interp_fraction
    );


/**
 * \brief Use the hardware interpolator to interpolate between two signed values.
 * 
 * \param low_level Low end of interpolation
 * \param high_level High end of interpolation
 * \param interp_fraction Interpolation fraction. Only last 8 bits are used.
 */
int32_t _interpolate_signed(
// inline int32_t _interpolate_signed(
    int32_t low_level,
    int32_t high_level,
    int32_t interp_fraction
    );

#endif