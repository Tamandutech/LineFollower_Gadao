// ativar ou desativar o debug
#define DEBUG_ARRAY             0
#define DEBUG_ARRAY_CALIBRAR    0
#define DEBUG_LTRAIS            0
#define DEBUG_ENCODER           0

// constantes para o PID
#define KP 0.05 // kp bom 0.04
#define KD 0.5    // kd bom 0.4
#define KI 0.0

// velocidade máxima
// vel boa 200
#define MAX_SPEED 200
#define MAX_SPEED_LINE 200
// velocidade de base
// vel boa 100
#define BASE_SPEED 100
#define BASE_SPEED_LINE 200

// numero de sensores, tempo de espera apos calibragem
#define NUM_SENSORS 6
#define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
#define EMITTER_PIN 11

// sensores da direita (parada) e esquerda (indicador de curva)
#define QRE_RIGHT A0
#define LEITURA_DIR 700
#define QRE_LEFT A7
#define LEITURA_ESQ 700

// quant. de indicadores a direita
#define STOP_INDICATORS 16

// pinos aos quais os motores estão ligados (direita)
#define RIGHT_MOTOR 4
#define RIGHT_MOTOR_PWM 5

// pinos aos quais os motores estão ligados (esquerda)
#define LEFT_MOTOR 7
#define LEFT_MOTOR_PWM 6

// LED indicador
#define LED 13
#define TIME_LED 300