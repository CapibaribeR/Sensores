#include "mbed.h"

#define MAX_VCC          3.3
#define RESOLUTION       65531.0
#define CSCA_SENSITIVITY 0.010
#define POINTS           500

// ---Pin settings
AnalogIn sens_pin(PA_1);

// ---Global variables
static float adc_to_current = MAX_VCC / (RESOLUTION * CSCA_SENSITIVITY);
static int p25 = int(POINTS * 0.25);
static int p75 = int(POINTS * 0.75);
static int turns = 4;  // Number of wire turns around sensor

// ---Function headers
uint16_t get_offset(void);
uint16_t treat_data(uint16_t data[POINTS]);
void sort_array(uint16_t arr[POINTS]);

int main(void)
{
    uint16_t pin_val[POINTS], pin_avg;
    uint16_t offset = get_offset(),
        i = 1;

    printf("\n\t---Sensor CSCA400---\n");

    while (true) {
        pin_val[i] = sens_pin.read_u16();
        i++;

        if (i == POINTS){
            pin_avg = treat_data(pin_val);
            printf("%f", abs(pin_avg - offset) * adc_to_current / turns);
            i = 0;
        }
    }
}

void sort_array(uint16_t arr[POINTS])
{
    static uint16_t i, j, a;
    for (i = 0; i < POINTS; ++i){
        for (j = i + 1; j < POINTS; ++j){
            if (arr[i] > arr[j]){
                a = arr[i];
                arr[i] = arr[j];
                arr[j] = a;
            }
        }
    }
}

uint16_t treat_data(uint16_t data[POINTS])
{
    uint16_t data_avg = 0;
    sort_array(data);

    for (int i = p25; i <= p75; i++){
        data_avg += data[i];
    }

    return data_avg / POINTS;
}

uint16_t get_offset(void)
{
    uint16_t data[POINTS];

    for (uint16_t i = 0; i < POINTS; i++){
        data[i] = sens_pin.read_u16();
    }

    return treat_data(data);
}
