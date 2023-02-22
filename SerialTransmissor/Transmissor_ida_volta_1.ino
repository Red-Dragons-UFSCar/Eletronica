// Comunicação Funcional Python-Transmissor-Receptor 12-10-2022

// Importação de Bibliotecas para comunicação

#include <esp_now.h>
#include <WiFi.h>


int estado = 0;                   // Contador de palavras recebidas da mensagem
bool start_message = false;       // Verificador de mensagem válida

const int senha = 111;            // Inicio da mensagem
const int senha_fim = 112;        // Fim da mensagem

int ledState = HIGH;

double t1, t2 = 0;                // Variaveis para medida de tempo
uint8_t v1;
uint8_t v2;

uint8_t broadcastAddress[] = {0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;


// Define uma estrutura de dados padrão de mensagem
typedef struct struct_message {
  bool dataAvaliable = false;
  const static int lengthVector = 10;
  uint8_t RD[lengthVector];
} struct_message;

typedef struct return_message {
  bool dataAvaliable = false;
  uint8_t RD_R[2] = {0,0};
} return_message;

// Cria a estrutura de dados para mensagem
struct_message myData;
return_message ReturnData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void  OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&ReturnData, incomingData, sizeof(ReturnData));
}
 



void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);

  // Inicio da marcação de tempo
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  //Serial.println("Salve");
  WiFi.mode(WIFI_STA);

  esp_now_init();

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}



void  serial(){
  /*
   * Função utilizada para a communicação Serial entre Python-ESP
   */

  // Se a porta serial está disponível
  while(Serial.available()){
    
    uint8_t temp1 = Serial.read();    // Leitura de uma palavra da mensagem em int
    String temp = String(temp1);      // Conversão para string - Apenas para print no terminal

    if(!start_message){                           // Se a mensagem não foi iniciada

      if(temp1 == senha){                           // Se a palavra lida for a senha
          //Serial.println("Inicio da mensagem");
          //Serial.println(temp);
          //Serial.println("---");
          myData.RD[estado] = temp1;                // Armazenamento da palavra na estrutura de dados na primeira posição
          estado++;                                 // Incremento da posição da mensagem
          start_message = true;                     // Inicio da leitura da mensagem
      }
      else{                                         // Caso a palavra lida não for a senha
        estado = 0;                                 // Mantém esperando receber a senha
      }
      
    }
    else{                                         // Se a mensagem já foi iniciada
      
      myData.RD[estado] = temp1;                    // Armazenamento da palavra na posição respectiva da mensagem
      estado++;                                     // Incremento da posicao da mensagem

      //Serial.println(temp);  
      
      if (estado == myData.lengthVector){                            // Se o contador de palavras chegar no tamanho da palavra  
        start_message = false;                      // Finaliza a mensagem
        estado=0;                                   // Zera o contador de palavras
        t2 = millis();
        double dt = t2-t1;                          // Mede o intervalo de tempo de leitura

        //Serial.print("Tempo: ");
        //Serial.println(dt);

        if(temp1==senha_fim){                       // Verifica se a ultima palavra da mensagem é a senha final
          //Serial.println("Mensagem Valida");
          myData.dataAvaliable = true;
        } else {
          //Serial.println("Mensagem Invalida");
        }
        
        t1 = t2;                                    // Reseta o timer

         esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
          v1 = ReturnData.RD_R[0];
          v2 = ReturnData.RD_R[1];
          //Serial.println(v1);
          //Serial.println(v2);
          Serial.write(v1);
          Serial.write(v2);

        
      }
      
    }
    
  }
}


void loop() {
  serial();
}