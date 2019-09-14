#include <Atributos.h>

void speedCondition() {
  // prevent the motor from going beyond max speed
  
  if(rightMotorSpeed > MAX_SPEED) rightMotorSpeed = MAX_SPEED;
  if(leftMotorSpeed > MAX_SPEED) leftMotorSpeed = MAX_SPEED;

  // keep the motor speed positive
  if(rightMotorSpeed < 0) rightMotorSpeed = 5;
  if(leftMotorSpeed < 0)  leftMotorSpeed = 5;
}
