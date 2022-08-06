#include "mbed.h"
#include "time.h"

AnalogIn sinal_in(PA_0);

int valor_entrada = 0; //irá receber a entrada do sensor
int valor_max = 45500; //valor máximo atingido pelo sensor durante os testes 
int valor_min = 6129; //valor mínimo atingido pelo sensor durante os testes
int angulo; //saída formatada em ângulo(0° - 120°)
unsigned long int prevTime = 0; //váriaveis para contagem de tempo
unsigned long int prevTime1 = 0; 
double erro_min = 4500; //limite inferior do valor_entrada para que não seja erro
double erro_max = 48000; //limite superior do valor_entrada para que não seja erro
bool erro = false; //varável que guarda a informação caso ocorra algum erro


//Mesma função map do arduino
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//Função millis() do arduino para contagem de tempo
double millis(){
    using namespace std::chrono;
    auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
    long micros = now_ms.time_since_epoch().count();
    return micros / 1000.0;
}

//Verificar se houve um curto ou se foi aberto o circuito do sensor
void Verifica_Erro(int entrada){
  if(entrada < erro_min or entrada > erro_max){
    if(millis() - prevTime >= 100){
       prevTime =  millis();
       erro = true;
    } 
  }
  else{
    prevTime = millis();
  }
}


int main()
{
    while (true) {
        valor_entrada = sinal_in.read_u16();

        Verifica_Erro(valor_entrada);

        angulo = map(valor_entrada,valor_min, valor_max, 0, 120);

        if(erro){
            printf("ERRO\n");
        }
        else{
            if(millis() - prevTime1 >= 200){
                printf("Valor entrada: ");
                printf("%d\n", valor_entrada);
                printf("Max: ");
                printf("%d\n", valor_max);
                printf("Min: ");
                printf("%d\n", valor_min);
                printf("Angulo: ");
                printf("%d", angulo);
                prevTime1 = millis();
            }
        }


    }
}