#include "mbed.h"

// Define as portas de Entrada e Saída
AnalogIn apps1(PA_0);
AnalogIn apps2(PA_1);
AnalogOut saida(PA_4);

// Declaração de Variáveis
int cont = 0;                      // Variável para denotar o início ou fim da contagem de tempo
unsigned long jikan = 0.0;         // Variável para guardar o número de milissegundos 
float aux = 0.0;                   // Variável para guardar o torque
int erro = 0;                      // Variável para denotar a existência de erro


// Função que determina quantos milissegundo se passaram até o momento
double millis(){
    using namespace std::chrono;
    auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
    long micros = now_ms.time_since_epoch().count();
    return micros / 1000.0;
}

// Função para verificar se ocorreu erros nos torques (Se os torques 
// se tiverem uma diferença de 10% durando 100 milissegundos, será erro)
int VerificaErroTorque(float t1, float t2) {
  if (abs(t1 - t2) > (0.1 * max(t1, t2))){
    cont += 1;
    if(cont == 1) {
      jikan = millis(); 
    }
    if(millis() - jikan > 100) {
      return 0;
    }
    return 1;
  } else{
      cont = 0;
      return 1;
  }
} 

// Inicia o laço principal
int main() {
  while(1) {
    // Determina a tensão que será enviada para o inversor
    // (read_u16 devolve valor entre 0 e 65535)
    int torque1 = apps1.read_u16();
    int torque2 = apps2.read_u16();
    int maxi = max(torque1, torque2);

    // Verifica Erros e Envia o Torque, se acontecerem os erros, deve mandar torque 0 para o inversor
    if (VerificaErroTorque(torque1, torque2) == 0){
      // Aconteceu erro (torques com mais de 10% de diferença por 100 mseg ou mais)
      saida.write_u16(0);
      aux = 0;
    } else{
      // Como o erro não aconteceu, o maior torque entre os sensores deve ser enviado para o inversor,
      // porém, como o inversor não funciona se o torque inicial não começar no mínimo aceitável por
      // ele, é preciso que o torque aumente ou diminua gradativamente até o torque desejado
      if (aux < maxi) {
        // aux deve ir aumentado gradativamente até o máximo, mas se der erro enquanto isso, deverá parar
        while ((aux < maxi) && (erro == 0)){
          if (VerificaErroTorque(apps1.read_u16(), apps2.read_u16()) == 0){
            // Aconteceu erro durante
            saida.write_u16(0);
            aux = 0;
            erro = 1;
          } else{
            // Manda o torque para o inversor, indo até o maior dos dois torques
            saida.write_u16(aux);
            aux = aux + 0.5;
          }
        }
      aux = max(apps1.read_u16(), apps2.read_u16());
      saida.write_u16(aux);
      } 
      if (aux > maxi) {
        // aux deve ir diminuindo gradativamente até o máximo, mas se der erro enquanto isso, deverá parar
        while ((aux > maxi) && (erro == 0)){
          if (VerificaErroTorque(apps1.read_u16(), apps2.read_u16()) == 0){
            // Aconteceu erro durante
            saida.write_u16(0);
            aux = 0;
            erro = 1;
          } else{
            // Manda o torque para o inversor, indo até o maior dos dois torques
            saida.write_u16(aux);
            aux = aux - 0.5;
          }
        }
      }
    erro = 0;
    printf("Recomeço");
    }
  }
}