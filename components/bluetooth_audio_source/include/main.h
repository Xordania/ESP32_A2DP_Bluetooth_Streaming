#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct {
    float frequency;
    float amplitude;
    uint32_t phase;
    float sample_rate;
} sine_wave_config_t;