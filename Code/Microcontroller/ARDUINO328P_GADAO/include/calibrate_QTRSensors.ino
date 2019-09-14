#include <Atributos.h>
#include <QTRSensors.h>

void calibrate_QTRSensors() {
  
  unsigned long lastTime = 0;
  boolean ledStatus = HIGH;
  
  for(int i=0; i<400; i++)  {
    qtra.calibrate(); // make the calibration take about 10 seconds
    
    //faz o LED piscar durante a calibragem
    if(millis() >= lastTime){
      lastTime = millis() + TIME_LED;
      ledStatus = !ledStatus;
      digitalWrite(LED, ledStatus);  
    }    
  }
  digitalWrite(LED, HIGH);
  delay(2000);  //wait for 2s to position the bot before entering the main loop
}
