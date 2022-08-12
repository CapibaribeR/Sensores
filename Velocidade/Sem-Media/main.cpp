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
        sensor.fall(NULL); // Desativa o interrupt
        if (periodo_atual != 0){
            f = 1000000 * ((float)(contagem_pulso)/periodo_atual); // Calcula a frequência em Hz
        }
        else{
            f = 0;
        }

        // Reinicialização dos parâmetro do pulso
        contagem_pulso = 0;                          
        periodo_atual = 0;                  
        ultimo_t = t.elapsed_time().count();        
        sensor.fall(&contadorFrequenciaISR); // Ativa o interrupt

        printf("%d\n", int(f * 30)); // Converte a velocidade para RPM
        ThisThread::sleep_for(500ms);  // Espera para uma nova medição
    }
}

void contadorFrequenciaISR(){
    contagem_pulso++;                                     // Quantidade de pulsos que aconteceram
    periodo_atual += t.elapsed_time().count() - ultimo_t; // Tamanho do pulso em s
    ultimo_t = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}
