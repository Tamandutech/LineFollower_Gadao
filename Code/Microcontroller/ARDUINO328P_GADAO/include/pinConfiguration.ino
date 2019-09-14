#include <Atributos.h>

void pinConfiguration() {
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(QRE_LEFT, INPUT);
  pinMode(QRE_RIGHT, INPUT);
}
