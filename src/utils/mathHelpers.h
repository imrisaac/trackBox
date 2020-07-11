#ifndef mathHelpers_h
#define mathHelpers_h

#include <cmath>

using namespace std;

// Constrain a value to be within the range: low and high
template <class T>
T constrain_value(const T amt, const T low, const T high);

inline float constrain_float(const float amt, const float low, const float high){
  return constrain_value(amt, low, high);
}

inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high){
  return constrain_value(amt, low, high);
}

inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high){
  return constrain_value(amt, low, high);
}

#endif // math_helpers_h