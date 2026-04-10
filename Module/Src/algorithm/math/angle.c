#include <math.h>

#include "angle.h"

float mathAngleWrapPi(float angle_rad)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 2.0f * pi;

    float wrapped = fmodf(angle_rad + pi, two_pi);
    if (wrapped <= 0.0f) {
        wrapped += two_pi;
    }

    return wrapped - pi;
}
