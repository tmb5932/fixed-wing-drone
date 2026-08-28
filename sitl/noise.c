#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "noise.h"

#define TWO_PI 6.28318530718f

float gaussian_noise(float stddev)
{
    static bool have_spare = false;
    static float spare;

    if (stddev <= 0.0f) return 0.0f;

    if (have_spare) {
        have_spare = false;
        return spare * stddev;
    }

    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = (float)rand() / (float)RAND_MAX;
    float mag = sqrtf(-2.0f * logf(u1));

    spare = mag * sinf(TWO_PI * u2);
    have_spare = true;
    return mag * cosf(TWO_PI * u2) * stddev;
}
