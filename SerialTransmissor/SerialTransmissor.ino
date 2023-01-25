// Comunicação Funcional Python-Transmissor-Receptor 12-10-2022

// Importação de Bibliotecas para comunicação
#include <WifiEspNowBroadcast.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

int estado = 0;                   // Contador de palavras recebidas da mensagem
bool start_message = false;       // Verificador de mensagem válida

const int senha = 111;            // Inicio da mensagem
const int senha_fim = 112;        // Fim da mensagem

int ledState = HIGH;

double t1, t2 = 0;                // Variaveis para medida de tempo


// Define uma estrutura de dados padrão de mensagem
typedef struct struct_message {
  bool dataAvaliable = false;
  const static int lengthVector = 10;
  uint8_t RD[lengthVector];
} struct_message;

 
// Cria a estrutura de dados para mensagem
struct_message myData;


void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);

  // Inicio da marcação de tempo
  t1 = millis();
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.println("Salve");

  // Transmissor
  WiFi.persistent(false);
  bool ok = WifiEspNowBroadcast.begin("ESPNOW", 3);
  if (!ok) {
    Serial.println("WifiEspNowBroadcast.begin() failed");
    //ESP.restart();
  }

  Serial.print("MAC address of this node is ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println("Press the button to send a message");
}

// Função que manda a mensagem para os robôs
void
sendMessage()
{
  char msg[60]; // Vetor para enviar a mensagem para os robôs

  if(myData.dataAvaliable){   // Se os dados estão sendo recebidos da serial
      // Enviar a mensagem por Broadcast
      int len = snprintf(msg, sizeof(msg), "hello ESP-NOW from %s at %lu",
                     WiFi.softAPmacAddress().c_str(), millis());
     
      WifiEspNowBroadcast.send(reinterpret_cast<const uint8_t*>(myData.RD), myData.lengthVector);
  
  }
  
  // Logger de informação, comentar abaixo se for necessário

  // /*

  Serial.println("Sending message");
  Serial.println(msg);
//  Serial.print("Recipients:");
//  const int MAX_PEERS = 20;
//  WifiEspNowPeerInfo peers[MAX_PEERS];
//  int nPeers = std::min(WifiEspNow.listPeers(peers, MAX_PEERS), MAX_PEERS);
//  for (int i = 0; i < nPeers; ++i) {
//    Serial.printf(" %02X:%02X:%02X:%02X:%02X:%02X", peers[i].mac[0], peers[i].mac[1],
//                  peers[i].mac[2], peers[i].mac[3], peers[i].mac[4], peers[i].mac[5]);
//  }
//  Serial.println();

  // */
}

void serial(){
  /*
   * Função utilizada para a communicação Serial entre Python-ESP
   */

  // Se a porta serial está disponível
  while(Serial.available()){
        
    uint8_t temp1 = Serial.read();    // Leitura de uma palavra da mensagem em int
    String temp = String(temp1);      // Conversão para string - Apenas para print no terminal

    if(!start_message){                           // Se a mensagem não foi iniciada

      if(temp1 == senha){                           // Se a palavra lida for a senha
          Serial.println("Inicio da mensagem");
          Serial.println(temp);
          Serial.println("---");
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

      Serial.println(temp);  
      
      if (estado == myData.lengthVector){                            // Se o contador de palavras chegar no tamanho da palavra  
        start_message = false;                      // Finaliza a mensagem
        estado=0;                                   // Zera o contador de palavras
        t2 = millis();
        double dt = t2-t1;                          // Mede o intervalo de tempo de leitura

        Serial.print("Tempo: ");
        Serial.println(dt);

        if(temp1==senha_fim){                       // Verifica se a ultima palavra da mensagem é a senha final
          Serial.println("Mensagem Valida");
          myData.dataAvaliable = true;
        } else {
          Serial.println("Mensagem Invalida");
        }
        
        t1 = t2;                                    // Reseta o timer

        sendMessage();
        
      }
      
    }
   
  }
}
 
void loop() {
  serial();
  WifiEspNowBroadcast.loop();
}
