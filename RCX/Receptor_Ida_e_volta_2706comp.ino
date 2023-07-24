// BIBLIOTECAS
//#include <esp_now.h>
//#include <WiFi.h>

hw_timer_t* timer = NULL;
#define ROBOT 3

// Bibliotecas para Comunicação ESP-NOW


#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Endereço de broadcast do Transmissor
uint8_t broadcastAddress[] = {0X40, 0x22, 0xD8, 0x03, 0xCE, 0xF0};
esp_now_peer_info_t peerInfo;


// Define uma estrutura de dados padrão de mensagem
typedef struct struct_message {
  // Variável para definir se a comunicação foi iniciada
  bool dataAvaliable = true;
  // Tamanho do vetor utilizado para receber a mensagem
  const static int lengthVector = 10;
  //  Vetor da mensagem
  uint8_t RD[lengthVector] = {111, 0, 0, 0, 0, 0, 0, 0, 0, 112};
  int cont = 0;
} struct_message;

typedef struct return_message {
  uint8_t ID = ROBOT;
  bool dataAvaliable = false;
  bool Last = true;
  uint8_t RD_R = 0;
} return_message;

// Cria a estrutura de dados para mensagem
struct_message myData;
return_message ReturnData;


// Variaveis para o LED(Comunicação)
#define  LED_PIN  2
int ledState = HIGH;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  /*
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  */
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.println("Recebi");
  digitalWrite(LED_PIN, ledState);
  ledState = 1 - ledState;
}
 


// DEFINIÇÃO DE PINOS
// Motores: motorxpin1 e motorxpim2 são utilizados para determinar a direção e o enable apenas para habilitar aquele motor na ponte h
// Motor 1
#define  motor1Pin1  27
#define motor1Pin2  26
#define enable1Pin  14
int direcao1[2] = {false, false};

// Motor 2
#define  motor2Pin1  12
#define  motor2Pin2  13
#define  enable2Pin  25
bool direcao2[2] = {false, false};

// PWM
#define  freq  30000
#define  pwmChannel1  0
#define  pwmChannel2  1
#define  resolution  8
volatile int dutyCycle = 240;
volatile int dutyCycle_b = 240;

// Encoder

//PINOS
#define  encoder_a  34
#define  encoder_b  35

//Leitrua Tensão
#define BAT_LED 5
#define READ 39
volatile int contador ; 

//Variaveis de leitura
volatile int  counterAB = 0;
volatile int  counterAB_b = 0;
double leitura = 0;

//Variaveis de calculos de velocidade
volatile double vel = 0;
volatile double vel_b = 0;


// CONTROLE
volatile int erro_a = 0;
volatile int erro_b = 0;
float somaerro_a = 0;
float somaerro_b = 0;
volatile double ap_a = 0;
volatile double ap_b = 0;
volatile double kp_a = 4;
volatile double ki_a = 0.04;
volatile int Escala_P = 1;
volatile int Escala_I = 1;
volatile int ref_a = 0;
volatile int  ref_b = 0;
volatile double ac_a = 0;
volatile double ac_b = 0;
volatile double eintegral_a = 0;
volatile double eintegral_b = 0;
int pwm_a;
int pwm_b;


//SOMADOR ENCODERS
void IRAM_ATTR ai1() {
  // Verificar possibilidade de retirar o if desta função
  counterAB ++;
  
}

void IRAM_ATTR ai2() {
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B    
  counterAB_b ++;
}

//Função para controlar os motores por meio do vetor utilizado na comunicação
void motor(int dta, int dtb) {
  ledcWrite(pwmChannel1, dta);
  ledcWrite(pwmChannel2, dtb);
  digitalWrite(motor1Pin1, direcao1[0]);
  digitalWrite(motor1Pin2, direcao1[1]);
  digitalWrite(motor2Pin1, direcao2[0]);
  digitalWrite(motor2Pin2, direcao2[1]);
}

void IRAM_ATTR onTime()
{
  // Extraindo informações dos motores dependedo de qual robô está sendo progamado
  //Jaocubo

  if (myData.dataAvaliable) {
    if(myData.RD[0] == 111){
      if (ROBOT == 1) {
        if(ref_a != (myData.RD[3])){
          eintegral_a = 0;
        }
        if (ref_b != (myData.RD[4])){
          eintegral_b = 0;
        }
        ref_a = myData.RD[3];
        ref_b = myData.RD[4];
        direcao1[0] = (*(myData.RD + 1) >> 7) & 1;
        direcao1[1] = (*(myData.RD + 1) >> 6) & 1;
        direcao2[0] = (*(myData.RD + 1) >> 5) & 1;
        direcao2[1] = (*(myData.RD + 1) >> 4) & 1;
      } else if (ROBOT == 2) {
        if(ref_a != myData.RD[5]){
          eintegral_a = 0;
        }
        if(ref_b != myData.RD[6]){
          eintegral_b =0;
        }
        ref_a = myData.RD[5];
        ref_b = myData.RD[6];
        direcao1[0] = (*(myData.RD + 1) >> 3) & 1;
        direcao1[1] = (*(myData.RD + 1) >> 2) & 1;
        direcao2[0] = (*(myData.RD + 1) >> 1) & 1;
        direcao2[1] = (*(myData.RD + 1) >> 0) & 1;
      } else if (ROBOT == 3) {
        if (ref_a != myData.RD[7]){
          eintegral_a =0;
        }
        if(ref_b != myData.RD[8]){
          eintegral_b = 0;
        }
        ref_a = myData.RD[7];
        ref_b = myData.RD[8];
        direcao1[0] = (*(myData.RD + 2) >> 7) & 1;
        direcao1[1] = (*(myData.RD + 2) >> 6) & 1;
        direcao2[0] = (*(myData.RD + 2) >> 5) & 1;
        direcao2[1] = (*(myData.RD + 2) >> 4) & 1;
      }

    
    else {
      ref_a = 0;
      ref_b = 0;
      direcao1[0] = false;
      direcao1[1] = false;
      direcao2[0] = false;
      direcao2[1] = false;
    }

    // Calculo de velocidade da roda A
    vel = counterAB * 1.8326;
    //Serial.print(counterAB);
    counterAB = 0;

    // Calculo de velocidade da roda B
    vel_b = counterAB_b * 1.8326;

    counterAB_b = 0;

    // Inicio controle
    // calculo dos erros
    erro_a = (ref_a - vel);
    erro_b = (ref_b - vel_b);

    //calculo das ações proporcionais
    ap_a = erro_a * kp_a;
    ap_b = erro_b * kp_a;

    //Calculo das ações integrativas
    eintegral_a = eintegral_a + erro_a * 0.01;
    eintegral_b = eintegral_b + erro_b * 0.01;
  

    // Mapeamento do erro
    ac_a = map(ap_a + eintegral_a * ki_a, -90, 90, -255, 255);
    ac_b = map(ap_b + eintegral_b * ki_a, -90, 90, -255, 255);

    // Correção do dutycycle após operações com o erro
    pwm_a = dutyCycle + ac_a;
    pwm_b = dutyCycle_b + ac_b;

    pwm_a = max(pwm_a, 0);
    pwm_a = min(pwm_a, 255);
    pwm_b = max(pwm_b, 0);
    pwm_b = min(pwm_b, 255);

    dutyCycle = pwm_a;
    dutyCycle_b = pwm_b;
    motor(dutyCycle,dutyCycle_b);
    } else if (myData.RD[0]== 112){
      Escala_P = myData.RD[1];
      Escala_I = myData.RD[2];
      if(ROBOT == 1){
          kp_a = myData.RD[3]*(pow(10,Escala_P*(-1)));
          ki_a = myData.RD[4]*(pow(10,Escala_P*(-1)));
      } else if (ROBOT ==2){
          kp_a = myData.RD[5]*(pow(10,Escala_P*(-1)));
          ki_a = myData.RD[6]*(pow(10,Escala_P*(-1)));
      } else if(ROBOT ==3){
          kp_a = myData.RD[7]*(pow(10,Escala_P*(-1)));
          ki_a = myData.RD[8]*(pow(10,Escala_P*(-1)));

      }
    }
  }
  contador++;
}


double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
}

void setup() {
  // Configurar os pinos como OUTPUTS
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  pinMode (encoder_a, INPUT);
  pinMode (encoder_b, INPUT);
  pinMode (BAT_LED, OUTPUT);

  analogSetAttenuation(ADC_11db);

  attachInterrupt(digitalPinToInterrupt(encoder_a), ai1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_b), ai2, RISING);

  // Configuração Funcionalidades do PWM
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // Anexar o canal do PWM na GPIO que será controlada
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  // Set up Serial Monitor
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  

  //TIMER SETUP
  timer = timerBegin(0,80, true);
  timerAttachInterrupt(timer, &onTime, true);
  timerAlarmWrite(timer,10000,true);
  timerAlarmEnable(timer);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_start());
  ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));


  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  
  
  esp_now_register_send_cb(OnDataSent);

}




//Função para obtenção de informações da comunicação, correção de erro e controle final dos motores.
//AVISO !! CASO SEJA NECESSARIO PRINTAR ALGUMA VARIÁVEL DESTE CAMPO CASO ELE SEJA USADO EM INTERRUPÇÃO UTILIZE O VOID LOOP


void loop()
{
  leitura = ReadVoltage(READ)*32;
  if(contador == 2000){
    if (leitura <= 78 ){
      digitalWrite(BAT_LED,HIGH);
    }
    ReturnData.RD_R = leitura;
    Serial.println(leitura);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ReturnData, sizeof(ReturnData));
    contador = 0;
  }

}