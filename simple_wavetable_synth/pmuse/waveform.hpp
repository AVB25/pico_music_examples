#include <vector>

#ifndef _WAVEFORM_H
#define _WAVEFORM_H
#ifndef _PICO_H
    #include "pico.h"
#endif

namespace pmuse{

/** Class template for storing and accessing the data in a single waveform.

 The length can be passed at runtime
 and the data can be changed at runtime. Can access he ```i```th
 sample by just calling ```wav[i]``` where ```wav``` is a ```Waveform<T>``` instance.
 */
template <typename T>
class Waveform
{
private:
    int32_t length;
    std::vector<T> vect;
    bool check_bounds = true;

public:
    Waveform(int32_t _length);
    Waveform(int32_t _length, T arr[]);
    ~Waveform();

    T &operator[](int32_t idx);

    int32_t get_length();
    bool set_check_bounds(bool _check_bounds);
    bool get_check_bounds();
};

/**8-bit waveform.
 Contains the data for a single waveform. The length can be passed at runtime
 and the data can be changed at runtime. Can access he ```i```th
 sample by just calling ```wav[i]``` where ```wav``` is an instance.
*/
typedef Waveform<int8_t> Waveform_i8;
}

#endif