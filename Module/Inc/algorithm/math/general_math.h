#pragma once

#include <stdint.h>
#include <stdbool.h>

#define STANDARD_GRAVITY_M_S2 9.80665f
#define DEG_TO_RAD 0.01745329251994329577f
#define RPM_TO_RADS 0.10471975511965977f

#define USER_PI 3.1415926535897932384f

bool mathClampf(float raw, float min, float max, float *out);
