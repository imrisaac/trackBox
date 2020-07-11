#ifndef __PI_UTILS_H_
#define __PI_UTILS_H_

// Constant representing PI
#define PI 3.1415926535897932384626433832795f

// Constant representing two PI
#define TWO_PI 6.283185307179586476925286766559f

// Constant representing PI over two
#define PI_OVER_TWO 1.5707963267948966192313216916398f

// Constant for converting radians to degrees
#define RAD_TO_DEG 57.295779513082320876798154814105f

// Constant for converting degrees to radians
#define DEG_TO_RAD 0.017453292519943295769236907684886f

// Constant for converting from degrees to radians
#define DegreesToRadians(x) (static_cast<float>(x) * DEG_TO_RAD)

// Constant conversion from radians to degrees
#define RadiansToDegrees(x) (static_cast<float>(x) * RAD_TO_DEG)

// Constant conversion from radians to Centidegrees
#define RadiansToCentiDegrees(x) (static_cast<float>(x) * RAD_TO_DEG * static_cast<float>(100))

#endif