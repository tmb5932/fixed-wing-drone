#ifndef SITL_NOISE_H
#define SITL_NOISE_H

// Box-Muller Gaussian sample, in the same units as stddev (0 stddev always
// returns exactly 0, so callers can pass a noise knob straight through
// without a separate on/off check). Shared by sitl_main.c and
// nav_sitl_main.c so both binaries' --noise/--seed flags behave identically.
float gaussian_noise(float stddev);

#endif // SITL_NOISE_H
