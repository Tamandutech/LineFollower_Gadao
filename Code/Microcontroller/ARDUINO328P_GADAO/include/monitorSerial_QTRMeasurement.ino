#include <Atributos.h>

void monitorSerial_QTRMeasurement()  {
  for(unsigned char i=0; i<NUM_SENSORS; i++)  {
    Serial.print(sensors[i]);
    Serial.print('\t');
  }
  //Serial.println(); //uncomment if you are using raw values
  Serial.print("Array: ");
  Serial.println(position);
}
