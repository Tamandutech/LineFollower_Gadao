#include <Atributos.h>

int PID()  {
  int error = 2500 - position;
  int p = error * KP;
  i = (i + error) * KI;
  int d = (error - lastError) * KD;
  lastError = error;
  return (p + i + d);
}
