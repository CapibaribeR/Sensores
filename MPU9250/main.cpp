#include "mbed.h"
#include "MPU9250.h"

MPU9250 mpu9250;

Timer t;
        
int main() {
  i2c.frequency(400000);  // Inicializa um I2C rápido (400Hz)  
  
  t.start();

  mpu9250.resetMPU9250(); // Reinicia os registradores para o default para preparar para a calibração do dispositivo
  mpu9250.MPU9250SelfTest(SelfTest); // Começa realizando um teste inicial e reporta os valores
  mpu9250.initMPU9250(); // Inicializa o MPU

  mpu9250.getAres(); // Sensitividade do acelerômetro
  mpu9250.getGres(); // Sensitividade do giroscópio

  while(true) {
    // Se o pino for para HIGh, todo os registradores recebem novos dados
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
  }
}
