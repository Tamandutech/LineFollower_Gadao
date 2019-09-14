#include <Atributos.h>

void monitor_SensoresLaterais() {
  Serial.print("Dir ");
  Serial.print(analogRead(QRE_RIGHT));
  Serial.print(" Esq ");
  Serial.print(analogRead(QRE_LEFT));
  Serial.print(" Counter: ");
  Serial.println(counter);
}
