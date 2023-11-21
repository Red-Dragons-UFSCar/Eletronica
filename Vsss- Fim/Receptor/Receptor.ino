hw_timer_t* timer = NULL;

// Bibliotecas para Comunicação ESP-NOW
#define WIFI_CHANNEL 13
#include <esp_now.h>
//#include <WiFi.h>
#include <esp_wifi.h>
#include "teste.h"
#include "Comm.cpp"
//#include "Controlador.cpp"


// CONTROLE (Antigo)
volatile int ErroAnterior1A = 0;
volatile int ErroAnterior1B = 0;
int ErroAtualA = 0;
int ErroAtualB = 0;
volatile int ref_a = 0;
volatile int ref_aAnterior = 0;
volatile int ref_b = 0;
volatile int ref_bAnterior = 0;
int SaidaA = 0;
int SaidaB =0;
int pwm_a;
int pwm_b;
volatile double AnteriorB = 0;
volatile double Anterior2B = 0;
volatile double AnteriorA = 0;
volatile double Anterior2A = 0;
int contA = 1;
int contB = 1;
int teste = 0;
int num = 0;
int num2 = 0;
volatile double speed1 = 0;
volatile double speed2 = 0;

// Controle novo 
const double Kc = 50;//279.8404 * 0.5;
const double Td = 0.00;
const double Ti = 0.03;//1.0132;
const double T = 0.01;
const double Ki = T*Kc/Ti;
const double Kd = Kc*Td/T;

volatile double error=0;
volatile double deltaU=0;
volatile double prevError1 = 0;
volatile double prev1Speed1 = 0;
volatile double prev2Speed1 = 0;
volatile int uM1 = 0;

volatile double error2=0;
volatile double deltaU2=0;
volatile double prevError2 = 0;
volatile double prev1Speed2 = 0;
volatile double prev2Speed2 = 0;
volatile int uM2 = 0;

volatile int contadorRef = 0; // Auxiliar para troca de direcao

// Filtro passa baixa
const double alpha = 0.85;
volatile int prevCounterAB = 0;
volatile int filterCounterAB= 0;
volatile int  counterAB = 0;
volatile int  counterAB_b = 0;

const double alpha_b = 0.85;
volatile int prevCounterAB_b = 0;
volatile int filterCounterAB_b= 0;

// Tratamento de troca de direcao A
volatile int signRefA = 0;
volatile int signRefAnteriorA = 0;
volatile int trocouDirecaoA = 0;
volatile int contadorCiclosA = 0;

// Tratamento de troca de direcao B
volatile int signRefB = 0;
volatile int signRefAnteriorB = 0;
volatile int trocouDirecaoB = 0;
volatile int contadorCiclosB = 0;

// Tratamento de arranque
volatile int saturador=0;

int direcao1[2] = {false, false};

int direcao2[2] = {false, false};

int leitura = 0;

int contador = 0;



// Endereço de broadcast do Transmissor
uint8_t broadcastAddress[] = {0X40, 0x22, 0xD8, 0x03, 0xCE, 0xF0};
esp_now_peer_info_t peerInfo;

// Cria a estrutura de dados para mensagem
struct_message myData;
return_message ReturnData;

int ledState = HIGH;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.println("Recebi");
  digitalWrite(LED_PIN, ledState);
  ledState = 1 - ledState;
}
 

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
  
  if (myData.dataAvaliable) {
    if(myData.RD[0] == 111){
      if (ROBOT == 1) {
        ref_a = myData.RD[3];
        ref_b = myData.RD[4];
        direcao1[0] = (*(myData.RD + 1) >> 7) & 1;
        direcao1[1] = (*(myData.RD + 1) >> 6) & 1;
        direcao2[0] = (*(myData.RD + 1) >> 5) & 1;
        direcao2[1] = (*(myData.RD + 1) >> 4) & 1;
      } else if (ROBOT == 2) {
        
        ref_a = myData.RD[5];
        ref_b = myData.RD[6];
        num = ref_a;
        teste=1;
        direcao1[0] = (*(myData.RD + 1) >> 3) & 1;
        direcao1[1] = (*(myData.RD + 1) >> 2) & 1;
        direcao2[0] = (*(myData.RD + 1) >> 1) & 1;
        direcao2[1] = (*(myData.RD + 1) >> 0) & 1;
      } else if (ROBOT == 3) {
        ref_a = myData.RD[7];
        ref_b = myData.RD[8];
        direcao1[0] = (*(myData.RD + 2) >> 7) & 1;
        direcao1[1] = (*(myData.RD + 2) >> 6) & 1;
        direcao2[0] = (*(myData.RD + 2) >> 5) & 1;
        direcao2[1] = (*(myData.RD + 2) >> 4) & 1;
      }

    speed1 = counterAB*CONV_ENC;
    speed2 = counterAB_b*CONV_ENC;

    // Verificar troca de direção motor A
    signRefA = (ref_a > 0) - (ref_a < 0);
    signRefAnteriorA = (ref_aAnterior > 0) - (ref_aAnterior < 0);

    // Condições para realizar a troca de direção
    //if((signRefA != signRefAnteriorA) && (contadorCiclosA>=10) ){
    //  trocouDirecaoA = 1;
    //}

    // Trava de tempo para a troca de direções
    if(contadorCiclosA<11){
      contadorCiclosA++;
    }
    
    if(trocouDirecaoA){
      ref_a=-20;    // Referencia menor para desacelerar
      if(speed1 < 15){ // Com velocidade pequena, ele permite a troca
        trocouDirecaoA=0;
        contadorCiclosA=0;
      }
    }

    // Verificar troca de direção motor B
    signRefB = (ref_b > 0) - (ref_b < 0);
    signRefAnteriorB = (ref_bAnterior > 0) - (ref_bAnterior < 0);

    // Condições para realizar a troca de direção
    if((signRefB != signRefAnteriorB) && (contadorCiclosB>=10) ){
      trocouDirecaoB = 1;
    }

    // Trava de tempo para a troca de direções
    if(contadorCiclosB<11){
      contadorCiclosB++;
    }
    
    if(trocouDirecaoB){
      ref_b=-20;  // Referencia menor para desacelerar
      if(speed2 < 15){ // Com velocidade pequena, ele permite a troca
        trocouDirecaoB=0;
        contadorCiclosB=0;
      }
    }

    ref_aAnterior = ref_a;
    ref_bAnterior = ref_b;

    // Adequação para definir a referência a partir da direção B*
    if( direcao1[0] && !direcao1[1] ){
      ref_a = ref_a*(-1);
    }

    // Se ele permite trocar a direção A
    if(!trocouDirecaoA){
    if(ref_a > 0){
      direcao1[0] = 0;
      direcao1[1] = 1;
    } else if(ref_a < 0){
      direcao1[0] = 1;
      direcao1[1] = 0;
      ref_a = ref_a*(-1);
    } else{
      direcao1[0] = 1;
      direcao1[1] = 1;
      ref_a = ref_a*(-1);
      }
    }

    // Adequação para definir a referência a partir da direção B*
    if( direcao2[0] && !direcao2[1] ){
      ref_b = ref_b*(-1);
    }

    // Se ele permite trocar a direção B
    if(!trocouDirecaoB){
    if(ref_b > 0){
      direcao2[0] = 0;
      direcao2[1] = 1;
    } else if (ref_b < 0){
      direcao2[0] = 1;
      direcao2[1] = 0;
      ref_b = ref_b*(-1);
    } else{
      direcao2[1] = 1;
      direcao2[1] = 1;
    }
    }
    
    // Filtro passa-baixa
    //filterCounterAB = alpha*counterAB + (1-alpha)*prevCounterAB;
    //prevCounterAB = counterAB;
    //counterAB = filterCounterAB;

    //filterCounterAB_b = alpha_b*counterAB_b + (1-alpha_b)*prevCounterAB_b;
    //prevCounterAB_b = counterAB_b;
    //counterAB_b = filterCounterAB_b;

    // motor 1
    speed1 = counterAB*CONV_ENC;
    error = ref_a-speed1;
    deltaU = Kc*(error - prevError1) + error*Ki - Kd*(speed1 - 2*prev1Speed1 + prev2Speed1);
    deltaU = floor(deltaU);

    // Tratamento de arranque
    saturador = 200;
    if(deltaU > saturador){
      deltaU=saturador;
    } else if (deltaU < -saturador){
      deltaU = -saturador;
    }
    
    uM1 = uM1 + deltaU;

    // Saturador para ação de controle estar entre (0, 1024)
    if(uM1 > 1024){
      uM1 = 1024;
    } else if(uM1 < 0){
      uM1 = 0;
    }

    prevError1 = error;
    prev2Speed1 = prev1Speed1;
    prev1Speed1 = speed1;

    // motor 2
    speed2 = counterAB_b*CONV_ENC;
    error2 = ref_b-speed2;
    deltaU2 = Kc*(error2 - prevError2) + error2*Ki - Kd*(speed2 - 2*prev1Speed2 + prev2Speed2);
    deltaU2 = floor(deltaU2);

    // Tratamento de arranque
    saturador = 200;
    if(deltaU2 > saturador){
      deltaU2=saturador;
    } else if (deltaU2 < -saturador){
      deltaU2 = -saturador;
    }
    
    uM2 = uM2 + deltaU2;

    prevError2 = error2;
    prev2Speed2 = prev1Speed2;
    prev1Speed2 = speed2;

    // Saturador para ação de controle estar entre (0, 1024)
    if(uM2 > 1024){
      uM2 = 1024;
    } else if(uM2 < 0){
      uM2 = 0;
    }

    // Conversão de ação de controle para PWM
    pwm_a = int(map(uM1,0,1024,80,255));
    pwm_b = int(map(uM2,0,1024,80,255));
    motor(pwm_a,pwm_b);
    
    //teste = 1;
    //num = uM1;
    
    counterAB=0;
    counterAB_b=0;

    ref_aAnterior = ref_a;
    ref_bAnterior = ref_b;
    ErroAnterior1A = ErroAtualA;
    ErroAnterior1B = ErroAtualB;
    
    }
  }
  
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
  pinMode (LED_PIN_2, OUTPUT);

  analogSetAttenuation(ADC_11db);

  attachInterrupt(digitalPinToInterrupt(encoder_a), ai1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_b), ai2, CHANGE);

  // Configuração Funcionalidades do PWM
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // Anexar o canal do PWM na GPIO que será controlada
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  // Set up Serial Monitor
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  

  //TIMER SETUP
  timer = timerBegin(0,80, true);
  timerAttachInterrupt(timer, &onTime, true);
  timerAlarmWrite(timer,10000,true);
  timerAlarmEnable(timer);

  // COMUNICAÇÃO
  
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
  leitura = ReadVoltage(READ)*32;
  if(contador == 2000){
    if (leitura <= 78 ){
      digitalWrite(BAT_LED,HIGH);
    }
    ReturnData.RD_R = leitura;
    //Serial.println(leitura);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ReturnData, sizeof(ReturnData));
    contador = 0;
   
  }
  if(teste)
  {
    Serial.print("1: ");
    Serial.println(num);
    teste = 0;
    num = 0;
    num2=0;
  }
}
