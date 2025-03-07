#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <mega16a.h>
#include <stdint.h>

// Type definitions
typedef enum {
    WAITING_FOR_RISE,
    WAITING_FOR_FALL
} CaptureState;

typedef struct {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} ColorValues;

// Function prototypes
void color_sensor_init(void);
ColorValues color_sensor_read(void);

#endif /* COLOR_SENSOR_H */