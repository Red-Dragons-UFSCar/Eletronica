// BIBLIOTECAS
//#include <esp_now.h>
//#include <WiFi.h>

hw_timer_t* timer = NULL;

#define ROBOT 1

// Bibliotecas para Comunicação ESP-NOW
#include <WifiEspNowBroadcast.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif


// Define uma estrutura de dados padrão de mensagem
typedef struct struct_message {
  // Variável para definir se a comunicação foi iniciada
  bool dataAvaliable = false;
  // Tamanho do vetor utilizado para receber a mensagem
  const static int lengthVector = 10;
  //  Vetor da mensagem
  uint8_t RD[lengthVector] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int cont = 0;
} struct_message;


// Cria a estrutura de dados para mensagem
struct_message myData;

// Variaveis para o LED(Comunicação)
static const int LED_PIN = 2;
int ledState = HIGH;


// DEFINIÇÃO DE PINOS
// Motores: motorxpin1 e motorxpim2 são utilizados para determinar a direção e o enable apenas para habilitar aquele motor na ponte h
// Motor 1
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 14;
bool direcao1[2] = {true, false};

// Motor 2
const int motor2Pin1 = 12;
const int motor2Pin2 = 13;
const int enable2Pin = 25;
bool direcao2[2] = {true, false};

// PWM
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
volatile int dutyCycle = 240;
volatile int dutyCycle_b = 240;

// Encoder

//PINOS
const int encoder_a = 34;
const int encoder_b = 35;

//Variaveis de leitura
int long counterAB = 0;
int long counterAB_b = 0;

//Variaveis de calculos de velocidade
volatile double vel = 0;
volatile double vel_b = 0;


// CONTROLE
volatile double erro_a = 0;
volatile double erro_b = 0;
float somaerro_a = 0;
float somaerro_b = 0;
volatile double ap_a = 0;
volatile double ap_b = 0;
const double kp_a = 4;
const double kp_b = 4;
const double ki_a = 0.04;
const double ki_b = 0.04;
double ref_a = 0;
double ref_b = 0;
volatile double ac_a = 0;
volatile double ac_b = 0;
volatile double eintegral_a = 0;
volatile double eintegral_b = 0;
int pwm_a;
int pwm_b;

//Cubo
int cubo = 0;
bool jao = true;
int farofa = 0;

int t1 = millis();

// Função para avaliar se a comunicação está disponível
void
processRx(const uint8_t mac[WIFIESPNOW_ALEN], const uint8_t* buf, size_t count, void* arg)
{
  if (int (count) == myData.lengthVector) {
    for (size_t i = 0; i < count; ++i) {
      myData.RD[i] = buf[i];
    }
    if (myData.cont > 100) {
      myData.dataAvaliable = true;
    } else {
      myData.cont++;
    }



  } else {
    //Serial.println("Existe algum problema no tamanho da mensagem...");
  }
  digitalWrite(LED_PIN, ledState);
  ledState = 1 - ledState;
}

//Função para controlar os motores por meio do vetor utilizado na comunicação
void motor(uint8_t *ptr, int dta, int dtb) {
  ledcWrite(pwmChannel1, dta);
  ledcWrite(pwmChannel2, dtb);
  digitalWrite(motor1Pin1, direcao1[0]);
  digitalWrite(motor1Pin2, direcao1[1]);
  digitalWrite(motor2Pin1, direcao2[0]);
  digitalWrite(motor2Pin2, direcao2[1]);
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

  attachInterrupt(digitalPinToInterrupt(encoder_a), ai1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_b), ai2, CHANGE);

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
  //timer = timerBegin(0,80, true);
  //timerAttachInterrupt(timer, &onTime, true);
  //timerAlarmWrite(timer,10000,true);
  //timerAlarmEnable(timer);

  WiFi.persistent(false);
  bool ok = WifiEspNowBroadcast.begin("ESPNOW", 3);
  if (!ok) {
    // Ligar led de erro 
  }

  WifiEspNowBroadcast.onReceive(processRx, nullptr);

}

//SOMADOR ENCODERS
void ai1() {
  // Verificar possibilidade de retirar o if desta função
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal A
  if (digitalRead(encoder_a) == HIGH)
  {
    counterAB ++;
  }
}

void ai2() {
  // Incrementa ou decrementa o contador de acordo com a condição do sinal no canal B
  if (digitalRead(encoder_b) == HIGH)
  {
    counterAB_b ++;
  }
}


//Função para obtenção de informações da comunicação, correção de erro e controle final dos motores.
//AVISO !! CASO SEJA NECESSARIO PRINTAR ALGUMA VARIÁVEL DESTE CAMPO CASO ELE SEJA USADO EM INTERRUPÇÃO UTILIZE O VOID LOOP
void onTime()
{
  // Extraindo informações dos motores dependedo de qual robô está sendo progamado
  if (myData.dataAvaliable) {
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
      direcao1[0] = (*(myData.RD + 1) >> 2) & 1;
      direcao1[1] = (*(myData.RD + 1) >> 3) & 1;
      direcao2[0] = (*(myData.RD + 1) >> 0) & 1;
      direcao2[1] = (*(myData.RD + 1) >> 1) & 1;
    } else {
      ref_a = myData.RD[7];
      ref_b = myData.RD[8];
      direcao1[0] = (*(myData.RD + 2) >> 7) & 1;
      direcao1[1] = (*(myData.RD + 2) >> 6) & 1;
      direcao2[0] = (*(myData.RD + 2) >> 5) & 1;
      direcao2[1] = (*(myData.RD + 2) >> 4) & 1;
    }

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
  erro_a = ref_a - vel;
  erro_b = ref_b - vel_b;

  //calculo das ações proporcionais
  ap_a = erro_a * kp_a;
  ap_b = erro_b * kp_b;

  //Calculo das ações integrativas
  eintegral_a = eintegral_a + erro_a * 0.01;
  eintegral_b = eintegral_b + erro_b * 0.01;
  

  // Mapeamento do erro
  ac_a = map(ap_a + eintegral_a * ki_a, -90, 90, -255, 255);
  ac_b = map(ap_b + eintegral_b * ki_b, -90, 90, -255, 255);

  // Correção do dutycycle após operações com o erro
  pwm_a = dutyCycle + ac_a;
  pwm_b = dutyCycle_b + ac_b;

  pwm_a = max(pwm_a, 0);
  pwm_a = min(pwm_a, 255);
  pwm_b = max(pwm_b, 0);
  pwm_b = min(pwm_b, 255);

  dutyCycle = pwm_a;
  dutyCycle_b = pwm_b;

  // Terceirizar essa pika
  if (jao)
  {
    ref_a = 15;
    ref_b = 15;
    cubo++;
    if (cubo == 501)
    {
      eintegral_a = 0;
      eintegral_b = 0;
      jao = false;
    }
  }
  else if (!jao)
  {
    ref_a = 20;
    ref_b = 20;
    cubo--;
    if (cubo == 0)
    {
      eintegral_a = 0;
      eintegral_b = 0;
      jao = true;
    }
  }

  
  motor(myData.RD, dutyCycle, dutyCycle_b);


}

void loop()
{
  //digitalWrite(2,HIGH);

  int t2 = millis();
  if (t2 - t1 > 8) {
    Serial.println(t2 - t1);
    onTime();
    t1 = t2;
  }

  WifiEspNowBroadcast.loop();
}
