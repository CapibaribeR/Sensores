#include "mbed.h"

InterruptIn sensor(PB_1, PullNone);

Timer t;
uint8_t contagem_pulso = 0;
uint64_t periodo_atual = 0, ultimo_t = 0;
float f = 0;

void contadorFrequenciaISR();

int main(){
    t.start();
    while (true){
        sensor.fall(NULL);
        if (periodo_atual != 0){
            f = 1000000 * ((float)(contagem_pulso)/periodo_atual); //Hz
        }
        else{
            f = 0;
        }

        contagem_pulso = 0;                          
        periodo_atual = 0;                         
        ultimo_t = t.elapsed_time().count();        
        sensor.fall(&contadorFrequenciaISR);
        printf("%d\n", int(f * 30)); //RPM
        ThisThread::sleep_for(500ms);
    }
}

void contadorFrequenciaISR(){
    contagem_pulso++;
    periodo_atual += t.elapsed_time().count() - ultimo_t;
    ultimo_t = t.elapsed_time().count();        
}