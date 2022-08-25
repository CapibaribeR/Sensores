#include "mbed.h"
#include "MPU9250.h"

#define BUFFER_SIZE     50

InterruptIn sensor(PB_1, PullNone);

MPU9250 mpu9250;

void ticker5HzISR();
void ticker20HzISR();

void setupInterrupts();

void contadorFrequenciaISR();

Ticker ticker5Hz;
Ticker ticker20Hz;
CircularBuffer <int, BUFFER_SIZE> state_buffer;
uint8_t switch_state = 0x00;
int current_state = 0;

Timer t;
uint8_t contagem_pulso = 0;
uint64_t periodo_atual = 0, ultimo_t = 0;
float f = 0;

int main() {
    i2c.frequency(400000);  // Inicializa um I2C rápido (400Hz)  
  
    t.start();

    mpu9250.resetMPU9250(); // Reinicia os registradores para o default para preparar para a calibração do dispositivo
    mpu9250.MPU9250SelfTest(SelfTest); // Começa realizando um teste inicial e reporta os valores
    mpu9250.initMPU9250(); // Inicializa o MPU

    mpu9250.getAres(); // Sensitividade do acelerômetro
    mpu9250.getGres(); // Sensitividade do giroscópio

    setupInterrupts(); 

    t.start();

    while (true) {
        if (state_buffer.full()) {
            state_buffer.pop(current_state);
        }
        else {
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = 0;
        }
        switch (current_state) {
            case 0:
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
                break;
            case 1:
                if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // Na interrupção, checa se a data está prota para interromper
                
                    mpu9250.readAccelData(accelCount);  // Lê os valores x/y/z do adc   
                    // Calcula a aceleração em g's
                    ax = (float)accelCount[0] * aRes - accelBias[0];
                    ay = (float)accelCount[1] * aRes - accelBias[1];   
                    az = (float)accelCount[2] * aRes - accelBias[2];  
                
                    mpu9250.readGyroData(gyroCount);  // Lê os valores x/y/z do adc  
                    // Calcula a velocidade angular em graus por segundo
                    gx = (float)gyroCount[0] * gRes - gyroBias[0];
                    gy = (float)gyroCount[1] * gRes - gyroBias[1];  
                    gz = (float)gyroCount[2] * gRes - gyroBias[2];   
                }

                // Normalização e conversão dos valores obtidos
                mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);

                // Printa os dados de aceleração convertendo para m/s² e fazendo a conversão
                printf("ax = %f", ax * 9.81 - 0.15); 
                printf(" ay = %f", ay * 9.81 - 0.1); 
                printf(" az = %f  m/s²\n\r", az * 9.81 + 0.12);

                // Printa os dados de velocidade angular
                printf("gx = %f", gx); 
                printf(" gy = %f", gy); 
                printf(" gz = %f  rad/s\n\r", gz); 

                // Lê o valores adc de temperatura em Kelvin, converte para graus Celsius e printa
                tempCount = mpu9250.readTempData();
                temperature = ((float) tempCount) / 333.87f + 21.0f;
                printf(" temperature = %f  C\n\r", temperature); 
                break;
        }
    }
}

void ticker5HzISR() {
    state_buffer.push(0);
}

void ticker20HzISR() {
    state_buffer.push(1);
}

void setupInterrupts() {
    ticker5Hz.attach(&ticker5HzISR, 200ms);
    ticker20Hz.attach(&ticker20HzISR, 50ms);
}

void contadorFrequenciaISR(){
    contagem_pulso++;                                     // Quantidade de pulsos que aconteceram
    periodo_atual += t.elapsed_time().count() - ultimo_t; // Tamanho do pulso em s
    ultimo_t = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}