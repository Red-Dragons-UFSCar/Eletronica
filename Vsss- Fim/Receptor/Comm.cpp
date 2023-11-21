#include "teste.h"

typedef struct struct_message {
  // Variável para definir se a comunicação foi iniciada
  bool dataAvaliable = false;  // true não precisa do transmisssor 
  // Tamanho do vetor utilizado para receber a mensagem
  //  Vetor da mensagem
  //uint8_t RD[lengthVector] = {111, 170, 160, 50, 50, 50, 50, 50, 50, 113};
  uint8_t RD[10] = {111, 0, 0, 0, 0, 0, 0, 0, 0, 0};
} struct_message;

// Estrutura para mensagem de retorno
typedef struct  return_message {
  //Identificador do robo
  uint8_t ID = ROBOT;
  //Se a mensagem está disponível
  bool dataAvaliable = false;
  bool Last = true;
  //Mensagem a ser retornada
  uint8_t RD_R = 0;
} return_message;