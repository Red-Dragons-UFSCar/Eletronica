hw_timer_t* timer = NULL;
#define ROBOT 3

// Bibliotecas para Comunicação ESP-NOW
#define WIFI_CHANNEL 13
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>


// Variaveis para o LED(Comunicação)
#define  LED_PIN  2
int ledState = HIGH;



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
int stateA = 0;
int stateB = 0;
int teste = 0;


//Cubo 
int cubo = 0;
bool jao = true;
int farofa = 0;
bool livia = true;


//VETOR A SER ALTERADO  PARA TESTES OS VALORES DA POSIÇÃO 5 E 6 SÃO AS REFS
int RD[19] = {111,0,1,0,1,20,20,1,0,1,0,20,20,1,0,1,0,20,20};



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
void motor(int *ptr, int dta, int dtb, int state, int state2){
  ledcWrite(pwmChannel1, dta);   
  ledcWrite(pwmChannel2, dtb); 
  if(state2){
    if(state){
    digitalWrite(motor1Pin1,*(ptr+1));
    digitalWrite(motor1Pin2,*(ptr+2));
    digitalWrite(motor2Pin1,*(ptr+3));
    digitalWrite(motor2Pin2,*(ptr+4));
    }
    else{
    digitalWrite(motor1Pin1,*(ptr+7));
    digitalWrite(motor1Pin2,*(ptr+8));
    digitalWrite(motor2Pin1,*(ptr+9));
    digitalWrite(motor2Pin2,*(ptr+10));
    }
  }
  else{
    digitalWrite(motor1Pin1,1);
    digitalWrite(motor1Pin2,1);
    digitalWrite(motor2Pin1,1);
    digitalWrite(motor2Pin2,1);
  }
}

void IRAM_ATTR onTime()
{
   
    //Erro 
    ErroAtualA = ref_a/(3.48192/2) - counterAB; // PQ Q O COUNTER TA INDO ATE 15 só ??????????????????????
    
    counterAB = 0;
    ErroAtualA = min(ErroAtualA, 30);

    ErroAtualB = ref_b/(3.48192/2) - counterAB_b;
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



if(jao)//frente 
  {
    //mudar aqui a velocidade
    ref_a = 20;
    ref_b = 20;
    cubo++;
    if(cubo >100 && cubo <201){
      ref_a = 40;
      ref_b = 40;
    }
    else if(cubo == 201)//faixa -50 a 201 -> anda | faixa 201 a 251 -> parado
    {
    livia= false;
    }
    else if(cubo ==251){
    jao = false;
    livia=true;
    }
  }
  else if(!jao)//tras
  {
    ref_a = 20;
    ref_b = 20;
    cubo--;
    if(cubo <100 && cubo >0){
      ref_a = 40;
      ref_b = 40;
    }
    else if(cubo == 0)//faixa 251 a 0 -> anda | faixa 0 a -50 -> parado
    {
    livia= false;
    }
    else if(cubo == -50){
    jao = true;
    livia=true;
    }
  }
  motor(RD,pwm_a,pwm_b,jao,livia);






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
  timer = timerBegin(0,80, true);
  timerAttachInterrupt(timer, &onTime, true);
  timerAlarmWrite(timer,10000,true);
  timerAlarmEnable(timer);
  
 

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
  }
  if(teste)
  {
    Serial.println(counterAB);
    teste = 0;
  }
  
}