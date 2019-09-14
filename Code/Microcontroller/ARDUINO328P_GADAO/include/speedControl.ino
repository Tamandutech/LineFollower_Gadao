#include <Atributos.h>

void speedControl() {
  digitalWrite(RIGHT_MOTOR, LOW);
  analogWrite(RIGHT_MOTOR_PWM, rightMotorSpeed);
  digitalWrite(LEFT_MOTOR, LOW);
  analogWrite(LEFT_MOTOR_PWM, leftMotorSpeed);
}
