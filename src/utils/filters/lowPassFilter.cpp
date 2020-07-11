#include "lowPassFilter.hpp"

template <class T>
DigitalLPF<T>::DigitalLPF() {
  // built in initialization
  _output = T();
}

// add a new raw value to the filter, retrieve the filtered result
template <class T>
T DigitalLPF<T>::apply(const T &sample, float cutoff_freq, float dt) {
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        _output = sample;
        return _output;
    }
    float rc = 1.0f/(TWO_PI*cutoff_freq);
    alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);
    _output += (sample - _output) * alpha;
    return _output;
}

template <class T>
T DigitalLPF<T>::apply(const T &sample) {
    _output += (sample - _output) * alpha;
    return _output;
}

template <class T>
void DigitalLPF<T>::compute_alpha(float sample_freq, float cutoff_freq) {
    if (cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
        alpha = 1.0;
    } else {
        float dt = 1.0/sample_freq;
        float rc = 1.0f/(TWO_PI*cutoff_freq);
        alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);
    }
}

// get latest filtered value from filter (equal to the value returned by latest call to apply method)
template <class T>
const T &DigitalLPF<T>::get() const {
    return _output;
}

template <class T>
void DigitalLPF<T>::reset(T value) { 
    _output = value; 
}
    
// constructors
template <class T>
LowPassFilter<T>::LowPassFilter() :
    _cutoff_freq(0.0f) {}

template <class T>
LowPassFilter<T>::LowPassFilter(float cutoff_freq) :
    _cutoff_freq(cutoff_freq) {}

template <class T>
LowPassFilter<T>::LowPassFilter(float sample_freq, float cutoff_freq)
{
    SetCutoffFrequency(sample_freq, cutoff_freq);
}

// change parameters
template <class T>
void LowPassFilter<T>::SetCutoffFrequency(float cutoff_freq) {
    _cutoff_freq = cutoff_freq;
}

template <class T>
void LowPassFilter<T>::SetCutoffFrequency(float sample_freq, float cutoff_freq) {
    _cutoff_freq = cutoff_freq;
    _filter.compute_alpha(sample_freq, cutoff_freq);
}

// return the cutoff frequency
template <class T>
float LowPassFilter<T>::GetCutoffFrequency(void) const {
    return _cutoff_freq;
}

template <class T>
T LowPassFilter<T>::apply(T sample, float dt) {
    return _filter.apply(sample, _cutoff_freq, dt);
}

template <class T>
T LowPassFilter<T>::apply(T sample) {
    return _filter.apply(sample);
}

template <class T>
const T &LowPassFilter<T>::get() const {
    return _filter.get();
}

template <class T>
void LowPassFilter<T>::reset(T value) {
    _filter.reset(value);
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter<int>;
template class LowPassFilter<long>;
template class LowPassFilter<float>;
