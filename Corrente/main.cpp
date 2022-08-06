#include "mbed.h"

#define MAX_VCC          3.3
#define RESOLUTION       65531.0
#define CSCA_SENSITIVITY 0.010

// ---Hardware settings
AnalogIn sens_pin(PA_1);

// ---Global variables
static int points = 500;
static float adc_to_current = MAX_VCC / (RESOLUTION * CSCA_SENSITIVITY);

int get_offset(void)
{
    long avg = 0;

    for (int i = 0; i < points; i++) {
        avg += sens_pin.read_u16();
    }

    return round(avg/points);
}

int main(void)
{
    int adc_value;
    int offset = get_offset();
    float current_value;

    printf("\n\t---Sensor CSCA400---\n");

    while (true) {
        current_value = abs(sens_pin - offset) * adc_to_current;
        printf("%f\n", current_value);
    }
}