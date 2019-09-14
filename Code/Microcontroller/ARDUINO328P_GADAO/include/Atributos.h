//constantes para o PID
#define KP 0.025  
#define KD 0.15
#define KI 0

//velocidade máxima
#define MAX_SPEED 60 

//velocidade de base
#define BASE_SPEED 35

//numero de sensores, tempo de espera apos calibragem
#define NUM_SENSORS 6
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN 11

//sensores da direita (parada) e esquerda (indicador de curva)
#define QRE_RIGHT A0
#define LEITURA_DIR 600                
#define QRE_LEFT A7
#define LEITURA_ESQ 600

//quant. de indicadores a direita
#define STOP_INDICATORS 16        

//pinos aos quais os motores estão ligados (direita)
#define RIGHT_MOTOR 4
#define RIGHT_MOTOR_PWM 5

//pinos aos quais os motores estão ligados (esquerda)
#define LEFT_MOTOR 7
#define LEFT_MOTOR_PWM 6

//LED indicador
#define LED 13                
#define TIME_LED 300