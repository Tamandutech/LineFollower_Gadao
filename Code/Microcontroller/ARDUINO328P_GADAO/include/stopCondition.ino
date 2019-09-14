#include <Atributos.h>

void stopCondition()  {
  int stopSensorRead = analogRead(QRE_RIGHT);

  if(stopSensorRead < LEITURA_DIR && linhaPreta_Direita == true)
  {
    linhaPreta_Direita = false;
    counter++;
  }
  
  if(stopSensorRead > LEITURA_DIR)
  {
    linhaPreta_Direita = true;
  }
  
  if(counter >= STOP_INDICATORS) {
    digitalWrite(RIGHT_MOTOR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, BASE_SPEED);
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, BASE_SPEED);
    delay(300);
    digitalWrite(RIGHT_MOTOR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, 0);
    delay(15000);
  }

  /*
  unsigned long currentMillis = millis();
  stopSensorRead = analogRead(QRE_RIGHT);
  //Serial.println(stopSensorRead);

  if(stopSensorRead < 700 && (currentMillis - previousMillis >= interval))  {
    previousMillis = currentMillis;
    counter = counter + 1;
    //Serial.println(counter);
  
  }
  */
}
