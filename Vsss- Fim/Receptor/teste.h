#include <Arduino.h>

#define ROBOT 3

//Pinos de leds
#define  LED_PIN  2
#define  LED_PIN_2 19
#define BAT_LED 5

//pinos de Motores
#define  motor1Pin1  27
#define motor1Pin2  26
#define enable1Pin  14
//
#define  motor2Pin1  12
#define  motor2Pin2  13
#define  enable2Pin  25

//Configurações PWM
#define  freq  30000
#define  pwmChannel1  0
#define  pwmChannel2  1
#define  resolution  8    

//Pinos de leitura do encoder
#define  encoder_a  34
#define  encoder_b  35
//Valor de conversão de pulsos para velocidade
#define  CONV_ENC   1.74096

//Pino de leitura da tensão da bateria
#define READ 39   
