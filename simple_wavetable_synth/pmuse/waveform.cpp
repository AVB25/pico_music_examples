#ifndef _WAVEFORM_H
#include "waveform.hpp"

using namespace::pmuse;

template <typename T>
Waveform<T>::Waveform(int32_t _length){
    length = _length;
    vect = std::vector<T> ( _length);
}

template <typename T>
Waveform<T>::Waveform(int32_t _length, T arr[]){
    length = _length;
    vect = std::vector<T> ( _length);
    for (int8_t i = 0; i<length; i++){
        vect[i] = arr[i];
    }
}

template <typename T>
Waveform<T>::~Waveform(){}

/** By default, check that idx doesn't excede the size of the array.
 This check can be removed by setting attribute ```check_bounds``` to
 ```false```
 */
template <typename T>
T& Waveform<T>::operator[](int32_t idx){
    if (check_bounds) {
        return vect[idx % length];
    } else {
        return vect[idx];
    }
}

template <typename T>
int32_t Waveform<T>::get_length(){
    return length;
}

template class Waveform<int8_t>;

#endif