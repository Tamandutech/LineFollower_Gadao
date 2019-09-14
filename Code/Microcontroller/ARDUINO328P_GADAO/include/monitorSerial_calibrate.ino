#include <Atributos.h>

void monitorSerial_calibrate() {
  //print the calibration minimum values measured when emitters were on
  for(int i=0; i<NUM_SENSORS; i++)  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  //print the maximum values
  for(int i=0; i<NUM_SENSORS; i++)  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}
