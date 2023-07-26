// Comunicação Funcional Python-Transmissor-Receptor 12-10-2022

// Importação de Bibliotecas para comunicação

hw_timer_t* timer = NULL;

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 14

int estado = 0;                   // Contador de palavras recebidas da mensagem
bool start_message = false;       // Verificador de mensagem válida

const int senha = 111;            // Inicio da mensagem
const int senha_fim = 113;        // Fim da mensagem

int ledState = HIGH;

double t1, t2 = 0;                // Variaveis para medida de tempo
uint8_t v1_a;
uint8_t v1_b;
uint8_t v2_a;
uint8_t v2_b;
uint8_t v3_a;
uint8_t v3_b;

uint8_t broadcastAddress[] = {0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;


// Define uma estrutura de dados padrão de mensagem
typedef struct struct_message {
  bool dataAvaliable = false;
  const static int lengthVector = 10;
  uint8_t RD[lengthVector];
} struct_message;

typedef struct return_message {
  uint8_t ID ;
  bool dataAvaliable = false;
  bool Last = false;
  uint8_t RD_R = 0;
} return_message;

// Cria a estrutura de dados para mensagem de envio
struct_message myData;

//Cria as estruturas de dados para as mensagens de retorno
return_message Return; 
return_message ReturnData1;
return_message ReturnData2;
return_message ReturnData3;

//Cria um array de estruturas
return_message Robos[3] = {ReturnData1,ReturnData2,ReturnData3};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void  OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Return, incomingData, sizeof(Return));
  Robos[Return.ID - 1].dataAvaliable = Return.dataAvaliable;
  Robos[Return.ID - 1].Last = Return.Last;
  Robos[Return.ID - 1].RD_R = Return.RD_R;    
}
 
void  IRAM_ATTR serial(){
  /*
   * Função utilizada para a communicação Serial entre Python-ESP
   */
  // Se a porta serial está disponível
  while(Serial.available()){
    
    uint8_t temp1 = Serial.read();    // Leitura de uma palavra da mensagem em int
    String temp = String(temp1);      // Conversão para string - Apenas para print no terminal

    if(!start_message){                           // Se a mensagem não foi iniciada

      if(temp1 == senha){                           // Se a palavra lida for a senha
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
      if (estado == myData.lengthVector){                            // Se o contador de palavras chegar no tamanho da palavra  
        start_message = false;                      // Finaliza a mensagem
        estado=0;                                   // Zera o contador de palavras

        if(temp1==senha_fim){                       // Verifica se a ultima palavra da mensagem é a senha final
          myData.dataAvaliable = true;
        } else {
          //Serial.println("Mensagem Invalida");
        }
        //Envia a mensagem 
         esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
         Serial.write(senha);
         for (int i = 0;i<3;i++){
          Serial.write(Robos[i].RD_R);
         }
         Serial.write(senha_fim);
        
      }
         
    }
    
  }
}


void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);

  // Inicio da marcação de tempo
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  //Serial.println("Salve");

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

  esp_wifi_set_max_tx_power(84);

  esp_now_init();

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }

  //TIMER SETUP
  /*
  timer = timerBegin(0,80, true);
  timerAttachInterrupt(timer, &serial, true);
  timerAlarmWrite(timer,10000,true);
  timerAlarmEnable(timer);
  */
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  serial();
}