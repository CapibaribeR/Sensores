#include "mbed.h"

//Define as portas de Entrada
AnalogIn volante (PA_0);        //A0

//Inicia o laço principal
int main()
{
    while (true) {
        //Lê o valor do sensor (entre 0 65535)
        //e converte para valores entre 0 e 3.3
        float valor = volante.read_u16() * (3.3 / 65535.0);
        //Centraliza os valores
        float centro = (valor - 1.6);
        //Printa centro
        printf("%.2f\n", centro);
        //Se centro for maior que 0
        if(centro > 0){
            printf("Direita\n");
        }
        //Se centro for menor que 0
        if(centro < 0){
            printf("Esquerda\n");
        }
        wait_us(500000);
    }
}