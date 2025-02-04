#include <vector>

#ifndef _WAVEFORM_H
#define _WAVEFORM_H
#ifndef _PICO_H
    #include "pico.h"
#endif

namespace pmuse{

// Ideally Waveform would be a template, but that involves defining in the header.
// Currently only have one implementation, if this changes rethink.

/** Contains the data for a single waveform. Data is stored as 8-bit signed integers.

 The length can be passed at runtime
 and the data can be changed at runtime. Can access he ```i```th
 sample by just calling ```wav[i]``` where ```wav``` is a ```Waveform<T>``` instance.
 */
class Waveform_i8
{
private:
    int32_t length;
    std::vector<int8_t> vect;
    bool check_bounds = true;

public:
    Waveform_i8(int32_t _length);
    Waveform_i8(int32_t _length, int8_t arr[]);
    ~Waveform_i8();

    int8_t &operator[](int32_t idx);

    int8_t get_length();
    bool set_check_bounds(bool _check_bounds);
    bool get_check_bounds();
};

/**8-bit waveform.
 Contains the data for a single waveform. The length can be passed at runtime
 and the data can be changed at runtime. Can access he ```i```th
 sample by just calling ```wav[i]``` where ```wav``` is a ```Waveform<T>``` instance.
*/
// typedef Waveform<int8_t> Waveform8;
}

#endif