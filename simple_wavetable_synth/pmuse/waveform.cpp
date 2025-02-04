#ifndef _WAVEFORM_H
#include "waveform.hpp"

pmuse::Waveform_i8::Waveform_i8(int32_t _length){
    length = _length;
    vect = std::vector<int8_t> ( _length);
}


pmuse::Waveform_i8::Waveform_i8(int32_t _length, int8_t arr[]){
    length = _length;
    vect = std::vector<int8_t> ( _length);
    for (int8_t i = 0; i<length; i++){
        vect[i] = arr[i];
    }
}

pmuse::Waveform_i8::~Waveform_i8(){}

/** By default, check that idx doesn't excede the size of the array.
 This check can be removed by setting attribute ```check_bounds``` to
 ```false```
 */
int8_t& pmuse::Waveform_i8::operator[](int32_t idx){
    if (check_bounds) {
        return vect[idx % length];
    } else {
        return vect[idx];
    }
}

int8_t pmuse::Waveform_i8::get_length(){
    return length;
}

#endif