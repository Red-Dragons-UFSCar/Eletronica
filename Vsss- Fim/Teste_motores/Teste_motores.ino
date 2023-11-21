/* --------------------------------------------------------------------------------------------------------------------------------------------------------------

### CÓDIGO PARA TESTE DE MOTOR ###
###REALIZAR ALTERAÇÕES APENAS NA FAIXA DE TESTES###
O teste consiste na mudança de velocidade e direção em periodos de tempo, visando testar o controle e a presença de travamento mecânico.

###problemas e conclusoes possiveis###
1-velocidade constante em todo o teste: problema de leitura de encoder (mal contato, encoder queimado, etc...)
2-velocidades diferentes nas rodas: problema de leitura de um único encoder
3-movimento em apenas uma direção: travamento mecanico, problema na ponte H, mal contato...
4-robo realizando curvas leves: necessidade de ajuste no controle
4-nada aconteceu: rezar e chorar

*///----------------------------------------------------------------------------------------------------------------------------------------------------------------
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


//Variaveis de teste
int CONT = 0; //--> inteiro contador (tempo) de mudança de estados
bool DIREC = true; // --> booleana de direção do robo (1->frente || 0->tras)
bool TRAVA = true;// --> booleana de trava de movimento(1->movimenta || 0->rodas travadas)


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
  if(state2){//MOTOR LIVRE PARA MOVIMENTO

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
  else{//MOTOR TRAVADO
    digitalWrite(motor1Pin1,1);
    digitalWrite(motor1Pin2,1);
    digitalWrite(motor2Pin1,1);
    digitalWrite(motor2Pin2,1);
  }
}

void IRAM_ATTR onTime()
{
   // ####### INICIO DA FUNÇÃO DE CONTROLE  (ADICIONAR/ SUBSTITUIR A FUNÇÃO UTILIZADA)
   
    //Erro 
    ErroAtualA = ref_a/1.74096 - counterAB; 
    counterAB = 0;
    
   // ErroAtualA = min(ErroAtualA, 30);

    ErroAtualB = ref_b/1.74096 - counterAB_b;
    counterAB_b = 0;
   // ErroAtualB = min(ErroAtualB, 30);

    compA=ErroAtualA + ErroAnterior1A;
    
    if(compA > 59){
      SaidaA = SaidaAnteriorA;
    }
    else{
      AnteriorA = (KpA*ErroAnterior1A);
      Anterior2A = (ErroAtualA*Soma_A);
      SaidaA = Anterior2A - AnteriorA + SaidaAnteriorA;
      // Saida Max Mantendo erro= 67.2 (Erro Atual 29 / Erro Anterior 30)
      // Saida Max Inicial= 52 (Erro Atual 30 / Erro Anterior 0)

    }
    
    compB=ErroAtualB + ErroAnterior1B;
 
    if(compB > 59){
      SaidaB = SaidaAnteriorB;
    }
    else{
      AnteriorB = (KpB*ErroAnterior1B);
      Anterior2B = (ErroAtualB*Soma_B);
      SaidaB = Anterior2B - AnteriorB + SaidaAnteriorB;;
     }

    ErroAnterior1A = ErroAtualA;
    ErroAnterior1B = ErroAtualB;
    SaidaAnteriorA = SaidaA;
    SaidaAnteriorB = SaidaB;
    /*
    if(SaidaAnteriorA == 0){
    pwm_b = map(SaidaB,0,52,200,255);
    pwm_a = map(SaidaA,0,52,200,255);
    }
    else{
    pwm_b = map(SaidaB,0,52,80,255);
    pwm_a = map(SaidaA,0,52,80,255);
    }
    */
    pwm_b = map(SaidaB,0,52,80,255);
    pwm_a = map(SaidaA,0,52,80,255);

    pwm_a = max(pwm_a,0);
    pwm_a = min(pwm_a,255);
    pwm_b = max(pwm_b,0);
    pwm_b = min(pwm_b,255);
    teste=1;

    // ####### FIM DA FUNÇÃO DE CONTROLE



//FAIXA TESTE DE MOTORES:
/*
--------------------------------------------------------------------------------------------------------------------------------------------------
O teste se baseia em 2 ciclos de funcionamento similar, variando em 3 etapas:
-ESTADO 1{
  se movimenta na vel1 
}
-ESTADO 2{
  se movimenta na vel2 na faixa indicada
}
-ESTADO 3:{
 trava o movimento das rodas até a troca de movimento
}
-TROCA DE CICLO:
  altera a booleana de direção ('DIREC'), ativa movimentação ('TRAVA'= false) e altera o sentido do contador (soma||subtrai)
--------------------------------------------------------------------------------------------------------------------------------------------------
MUDANÇAS PADRÕES :
velocidade -> alterar valores de ref_a e ref_b
tempo de movimento na velocidade 1 -> alterar o primeiro valor de 'CONT' no if -> "ESTADO 2"
tempo de movimento na velocidade 2 -> alterar o segundo valor de 'CONT' no if -> "ESTADO 2" e if -> "Estado 3"
tempo travado -> alterar o valor de 'CONT' na troca de ciclo
*/

//CICLO 1
if(DIREC)//frente 
  {
    //ESTADO 1:
    //velocidade 1 (REFERENCIA 1 -- manter valores iguais em 'a' e 'b' para movimento reto)
    ref_a = 20;
    ref_b = 20;

    CONT++;

    if(CONT >100 && CONT <201){//ESTADO 2:
    //velocidade 2 (REFERENCIA 2)
      ref_a = 40;
      ref_b = 40;
    }

    else if(CONT == 201)//ESTADO 3:
    {
    ref_a = 0;
    ref_b = 0;
    TRAVA= false;
    }

    else if(CONT ==251){//TROCA DE CICLO 
    DIREC = false;
    TRAVA=true;
    }
  }
  //FIM DO CICLO 1
  //CICLO 2
  else if(!DIREC)//tras
  {
    //ESTADO 1:
    //velocidade 1 (REFERENCIA 1 -- manter valores iguais em 'a' e 'b' para movimento reto)
    ref_a = 20;
    ref_b = 20;
    CONT--;
    if(CONT <100 && CONT >0){//ESTADO 2:
    //velocidade 2 (REFERENCIA 2)
    ref_a = 40;
    ref_b = 40;
    }
    else if(CONT == 0)//ESTADO 3
    {
    ref_a = 0;
    ref_b = 0;
    TRAVA= false;
    }
    else if(CONT == -50){//TROCA DE CICLO
    DIREC = true;
    TRAVA=true;
    }
  }
  //FIM DO CICLO 2
  motor(RD,pwm_a,pwm_b,DIREC,TRAVA);//CHAMADA DA FUNÇÃO DO MOTOR

  //FIM DA FAIXA DE TESTES DE MOTORES
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
    Serial.println(pwm_a);
    teste = 0;
  }
  
}