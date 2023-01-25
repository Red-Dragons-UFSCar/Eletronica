/**
 * @file
 *
 * EspNowBroadcast.ino demonstrates how to perform ESP-NOW pseudo broadcast with @c WifiEspNowBroadcast .
 * You need two or more ESP8266 or ESP32 devices to run this example.
 *
 * All devices should run the same program.
 * You may need to modify the PIN numbers so that you can observe the effect.
 *
 * With the program running on several devices:
 * @li Press the button to transmit a message.
 * @li When a device receives a message, it will toggle its LED between "on" and "off" states.
 */

#include <WifiEspNowBroadcast.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif


// Define uma estrutura de dados padrão de mensagem
typedef struct struct_message {
  bool dataAvaliable = false;
  const static int lengthVector = 10;
  uint8_t RD[lengthVector];
} struct_message;


// Cria a estrutura de dados para mensagem
struct_message myData;

// DEFINIÇÃO DE PINOS
// Motor 1
const int motor1Pin1 = 27; 
const int motor1Pin2 = 26; 
const int enable1Pin = 14; 

// Motor 2
const int motor2Pin1 = 12;
const int motor2Pin2 = 13;
const int enable2Pin = 25; 

// PWM
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
volatile int dutyCycle = 240;
volatile int dutyCycle_b = 240;


/**
 * @brief PIN number of a button.
 *
 * The default `0` is the "flash" button on NodeMCU, Witty Cloud, Heltec WiFi_Kit_32, etc.
 */
static const int BUTTON_PIN = 0;

/**
 * @brief PIN number of an LED.
 *
 * The default `2` is the blue LED on ESP-12F.
 */
static const int LED_PIN = 2;

int ledState = HIGH;

int t1 = millis();

void
processRx(const uint8_t mac[WIFIESPNOW_ALEN], const uint8_t* buf, size_t count, void* arg)
{
  Serial.printf("Message from %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3],
                mac[4], mac[5]);
  if (int (count) == myData.lengthVector){
    for (size_t i = 0; i < count; ++i) {
      Serial.print(static_cast<uint8_t>(buf[i]));
      Serial.print(" ");
      myData.RD[i] = buf[i];
    }
    Serial.println();
  } else{
    Serial.println("Existe algum problema no tamanho da mensagem...");  
  }

  // Extraindo os bits de direção da segunda posição do vetor
  
  bool direcao1_M1 = (*(myData.RD+1)>>7)&1;
  bool direcao2_M1 = (*(myData.RD+1)>>6)&1;
  bool direcao1_M2 = (*(myData.RD+1)>>5)&1;
  bool direcao2_M2 = (*(myData.RD+1)>>4)&1;
  
  Serial.print("Direcao 1 = ");
  Serial.println(direcao1_M1);
  Serial.print("Direcao 2 = ");
  Serial.println(direcao2_M1);
  digitalWrite(LED_PIN, ledState);
  ledState = 1 - ledState;

  Serial.print("Velocidade 1 = ");
  Serial.println(int(myData.RD[3]));
  Serial.print("Velocidade 2 = ");
  Serial.println(int(myData.RD[4]));
  ledcWrite(pwmChannel1, int(myData.RD[3])); 
  digitalWrite( motor1Pin1 , direcao1_M1 );
  digitalWrite( motor1Pin2 , direcao2_M1 );

  ledcWrite(pwmChannel2, int(myData.RD[4])); 
  digitalWrite( motor2Pin1 , direcao1_M2 );
  digitalWrite( motor2Pin2 , direcao2_M2 );
}

void
setup()
{
  Serial.begin(115200);
  Serial.println();

  WiFi.persistent(false);
  bool ok = WifiEspNowBroadcast.begin("ESPNOW", 3);
  if (!ok) {
    Serial.println("WifiEspNowBroadcast.begin() failed");
    //ESP.restart();
  }
  // WifiEspNowBroadcast.begin() function sets WiFi to AP+STA mode.
  // The AP interface is also controlled by WifiEspNowBroadcast.
  // You may use the STA interface after calling WifiEspNowBroadcast.begin().
  // For best results, ensure all devices are using the same WiFi channel.

  WifiEspNowBroadcast.onReceive(processRx, nullptr);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);

  Serial.print("MAC address of this node is ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println("Press the button to send a message");

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

    // Configuração Funcionalidades do PWM
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  
  // Anexar o canal do PWM na GPIO que será controlada
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);
}

void
loop()
{
  int t2 = millis();

  //if(t2-t1>1000){
  //  sendMessage();
  //  t1 = t2;
  //}

  /*
  if (digitalRead(BUTTON_PIN) == LOW) { // button is pressed
    sendMessage();

    while (digitalRead(BUTTON_PIN) == LOW) // wait for button release
      ;
  }*/

  WifiEspNowBroadcast.loop();
  //delay(10);
}
