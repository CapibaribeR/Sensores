#include "mbed.h"

AnalogIn sensor (PA_0);        //A0
float media, soma = 0;
int i, j, vet[10];

float mediaVetor();

float mediaVetor(int vet[], int i){
  soma = 0;
  for(i=0; i<10; i++){
    soma = soma + vet[i];
  }
  media = (float) soma/i;

  return media;
}

int main(){
   while (true) {
       for(j=0; j<10; j++){
           if(sensor.read_u16()<1500){
               vet[j] = 0;
           }
           else{
               vet[j] = sensor.read_u16();
           }
       }

       float media = mediaVetor(vet,i);
       printf("RPM: %.2f\n", media * (2000.0 / 65535.0));
       printf("RPM_u16: %.2f\n", media);
       ThisThread::sleep_for(500ms);
   }
}