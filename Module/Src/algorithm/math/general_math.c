#include <stdbool.h>
#include <string.h>

#include "general_math.h"

bool mathClampf(float raw, float min, float max, float *out)
{
    if (min > max || out == NULL) {
        return false;
    }

    raw = (raw < min) ? min : raw;
    raw = (raw > max) ? max : raw;

    *out = raw;

    return true;
}