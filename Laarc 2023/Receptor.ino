hw_timer_t* timer = NULL;
#define ROBOT 3

// Bibliotecas para Comunicação ESP-NOW
#define WIFI_CHANNEL 13
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>


// Endereço de broadcast do Transmissor
uint8_t broadcastAddress[] = {0X40, 0x22, 0xD8, 0x03, 0xCE, 0xF0};
esp_now_peer_info_t peerInfo;


// Define uma estrutura de dados para padrão de mensagem recebida
typedef struct struct_message {
  // Variável para definir se a comunicação foi iniciada
  bool dataAvaliable = false;
  //  Vetor da mensagem
  uint8_t RD[10] = {111, 0, 0, 0, 0, 0, 0, 0, 0, 113};
} struct_message;

// Define uma estrutura de dados para padrão de mensagem retornada
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
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Mudar estado do LED ao receber mensagem de comunicação
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
volatile int dutyCycle = 0;
volatile int dutyCycle_b = 0;

// Encoder

//PINOS
#define  encoder_a  34
#define  encoder_b  35
//Contagem de Pulsos
volatile int  counterAB = 0;
volatile int  counterAB_b = 0;

//Leitrua Tensão
#define BAT_LED 5
#define READ 39
volatile int contador; 
double leitura = 0;

// CONTROLE
volatile int ErroAnterior1A = 0;
volatile int ErroAnterior1B = 0;
int ErroAtualA = 0;
int ErroAtualB = 0;
volatile int Escala_P1 = 0;
volatile int Escala_I1 = 0;
volatile int Escala_P2 = 0;
volatile int Escala_I2 = 0;
volatile int ref_a = 0;
volatile int ref_b = 0;
int SaidaA = 0;
int SaidaB =0;
int pwm_a;
int pwm_b;
volatile double KpA = 1.2;
volatile double KiA = 50;
volatile double KpB = 1.3;
volatile double KiB = 50;
volatile double Soma_A = KpA+(KiA/100);
volatile double Soma_B = KpB+(KiB/100);
volatile double SaidaAnteriorA = 0;
volatile double SaidaAnteriorB = 0;
volatile double AnteriorB = 0;
volatile double Anterior2B = 0;
volatile double AnteriorA = 0;
volatile double Anterior2A = 0;
int contA = 1;
int contB = 1;
volatile double compA;
volatile double compB;
volatile double valA;
volatile double valB;
int teste = 0;


//Função somadora dos contadores do encoder 1
void IRAM_ATTR ai1() {
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  counterAB ++;
}
//Função somadora dos contadores do encoder 2
void IRAM_ATTR ai2() {
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B    
  counterAB_b ++;
}

//Função para controlar os motores por meio do vetor utilizado na comunicação
void motor(int dta, int dtb) {
  //Altera os valores de pwm para controle DE VELOCIDADE dos motores
  ledcWrite(pwmChannel1, dta);
  ledcWrite(pwmChannel2, dtb);
  //Altera os valores digitais para controle DE DIREÇÃO dos motores
  digitalWrite(motor1Pin1, direcao1[0]);
  digitalWrite(motor1Pin2, direcao1[1]);
  digitalWrite(motor2Pin1, direcao2[0]);
  digitalWrite(motor2Pin2, direcao2[1]);
}

void IRAM_ATTR onTime()
{
  // Extraindo informações dos motores dependedo de qual robô está sendo progamado
  //Jaocubo

  if (myData.dataAvaliable) { //Verificar se alguma mensagem foi recebida
    if(myData.RD[0] == 111){ //Verificar integridade da mensagem pela senha
      if (ROBOT == 1) { //Caso 1: Robo 1
        ref_a = myData.RD[3];
        ref_b = myData.RD[4];
        direcao1[0] = (*(myData.RD + 1) >> 7) & 1;
        direcao1[1] = (*(myData.RD + 1) >> 6) & 1;
        direcao2[0] = (*(myData.RD + 1) >> 5) & 1;
        direcao2[1] = (*(myData.RD + 1) >> 4) & 1;
      } else if (ROBOT == 2) {//Caso 2: Robo 2
        
        ref_a = myData.RD[5];
        ref_b = myData.RD[6];
        direcao1[0] = (*(myData.RD + 1) >> 3) & 1;
        direcao1[1] = (*(myData.RD + 1) >> 2) & 1;
        direcao2[0] = (*(myData.RD + 1) >> 1) & 1;
        direcao2[1] = (*(myData.RD + 1) >> 0) & 1;
      } else if (ROBOT == 3) {//Caso 3: Robo 3
        ref_a = myData.RD[7];
        ref_b = myData.RD[8];
        direcao1[0] = (*(myData.RD + 2) >> 7) & 1;
        direcao1[1] = (*(myData.RD + 2) >> 6) & 1;
        direcao2[0] = (*(myData.RD + 2) >> 5) & 1;
        direcao2[1] = (*(myData.RD + 2) >> 4) & 1;
      }
      /*
      Inicio do controle 
      */ 
      //Calculo do erro da Roda A
      ErroAtualA = ref_a/1.74096 - counterAB; 
      counterAB = 0;
      ErroAtualA = min(ErroAtualA, 30);
      //Calculo do erro da Roda B
      ErroAtualB = ref_b/1.74096 - counterAB_b;
      counterAB_b = 0;
      ErroAtualB = min(ErroAtualB, 30);

      compA=ErroAtualA + ErroAnterior1A;
    
      if(compA > 50){
        SaidaA = SaidaAnteriorA;
      }
      else{
        AnteriorA = (KpA*ErroAnterior1A);
        Anterior2A = (ErroAtualA*Soma_A);
        //SaidaA = Anterior2A - AnteriorA + SaidaAnteriorA;
        SaidaA = Anterior2A - AnteriorA + SaidaAnteriorA;

      }

      valA=contA*19;
      if((SaidaA >= valA)){
        SaidaA = valA;
        contA++;
      }
      else{
        contA = 1;
      }
    
      compB=ErroAtualB + ErroAnterior1B;
 
      if(compB > 50){
        SaidaB = SaidaAnteriorB;
      }
      else{
        AnteriorB = (KpB*ErroAnterior1B);
        Anterior2B = (ErroAtualB*Soma_B);
        SaidaB = Anterior2B - AnteriorB + SaidaAnteriorB;;
      }
      valB=contB*19;
      if(SaidaB >= valB){
        SaidaB = valB;
        contB++;
      }
      else {
        contB = 1;
      }
      ErroAnterior1A = ErroAtualA;
      ErroAnterior1B = ErroAtualB;
      SaidaAnteriorA = SaidaA;
      SaidaAnteriorB = SaidaB;
    
      pwm_b = map(SaidaB,0,30,200,255);
      pwm_a = map(SaidaA,0,30,200,255);
      pwm_a = max(pwm_a,0);
      pwm_a = min(pwm_a,255);
      pwm_b = max(pwm_b,0);
      pwm_b = min(pwm_b,255);
      motor(pwm_a,pwm_b);
    
    } else if (myData.RD[0]== 112){
      Escala_P1 = myData.RD[5];
      Escala_I1 = myData.RD[6];
      Escala_P2 = myData.RD[7];
      Escala_I2 = myData.RD[8];
      KpA = myData.RD[1]*(pow(10,Escala_P1*(-1)));
      KiA = myData.RD[2]*(pow(10,Escala_I1*(-1)));
      KpB = myData.RD[3]*(pow(10,Escala_P2*(-1)));
      KiB = myData.RD[4]*(pow(10,Escala_I2*(-1)));
      Soma_A = KpA+(KiA/100);
      Soma_B = KpB+(KiB/100);
    }
  }
  
}

//Função para corrigir leitura da bateria
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
  //Atenuação para leitura da bateria
  analogSetAttenuation(ADC_11db);

  //Configuração das interrupções para leitura do Encoder 
  attachInterrupt(digitalPinToInterrupt(encoder_a), ai1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_b), ai2, CHANGE);

  // Configuração Funcionalidades do PWM
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // Anexar o canal do PWM na GPIO que será controlada
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  // Configurar o Serial
  Serial.begin(115200);
  //Configurar o led da dev kit
  pinMode(LED_PIN, OUTPUT);
  

  //Configuração do timer para interrupção do controle
  timer = timerBegin(0,80, true);
  timerAttachInterrupt(timer, &onTime, true);
  timerAlarmWrite(timer,10000,true);
  timerAlarmEnable(timer);
  
  //Configuração do ESP_NOW
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_start());
  ESP_ERROR_CHECK( esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
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
  //Leitura do pino de medida da bateria
  leitura = ReadVoltage(READ)*32;
  if(contador == 2000){
    if (leitura <= 78 ){
      digitalWrite(BAT_LED,HIGH);
    }
    ReturnData.RD_R = leitura;
    //Serial.println(leitura);
    //Envio da mensagem de retorno
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ReturnData, sizeof(ReturnData));
    contador = 0;
   
  }
}