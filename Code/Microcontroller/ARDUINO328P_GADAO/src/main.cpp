#include <Arduino.h>
#include <Atributos.h>

// biblioteca para o array
#include <QTRSensors.h>

// variaveis do array (valor de posicao sera recebida pela leitura do array)
unsigned int position = 0;
uint16_t sensors[NUM_SENSORS];

// variaveis dos sensores lateriais
unsigned int sensorDir = 0;
unsigned int sensorEsq = 0;
unsigned int sensorDirCount = 0;
unsigned int sensorEsqCount = 0;

// variaveis do PID
int lastError = 0;
int iPID = 0;

// variaveis para o controle de velocidade
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

// variaveis de velocidade do robo
int veloBase = BASE_SPEED;
int veloMax = MAX_SPEED;

boolean statusCurva = false;

// variaveis para o sensor da esquerda
boolean sensorEsqLinha = true;

// variaveis para o sensor da direita
boolean sensorDirLinha = true;

// pinos aos quais o array esta conectado, leitura digital (analogico tbm pode
// ler como digital)
QTRSensors array;

void sensorArray();
void sensorArrayCalibrar();
void sensorLaterais();
int PID();
void pinConfiguration();
void velocidadeCalc();
void velociadeCond();
void velociadeControl();
void stopCondition();

void setup() {
  array.setTypeAnalog();
  array.setSensorPins((const uint8_t[]){1, 2, 3, 4, 5, 6}, NUM_SENSORS);
  array.setEmitterPin(EMITTER_PIN);
  array.setSamplesPerSensor(NUM_SAMPLES_PER_SENSOR);

  pinConfiguration();
  sensorArrayCalibrar();
  Serial.begin(9600);
}

void loop() {
  sensorArray();
  sensorLaterais();

  velocidadeCalc();
  velociadeCond();
  velociadeControl();
  
  stopCondition();
}

void sensorArrayCalibrar() {

  unsigned long lastTime = 0;
  boolean ledStatus = HIGH;

  for (int i = 0; i < 100; i++) {
    array.calibrate(); // make the calibration take about 10 seconds

    // faz o LED piscar durante a calibragem
    if (millis() >= lastTime) {
      lastTime = millis() + TIME_LED;
      ledStatus = !ledStatus;
      digitalWrite(LED, ledStatus);
    }
  }
  digitalWrite(LED, HIGH);

#if DEBUG_ARRAY_CALIBRAR == 1
  // Imprime os valores minimos dos sensores
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(array.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Imprime os valores maximos dos sensores
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(array.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
#endif

  delay(2000);
}

void sensorArray() {
  position = array.readLineWhite(sensors);
#if DEBUG_ARRAY == 1
  for (unsigned char i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensors[i]);
    Serial.print('\t');
  }
  // Serial.println(); //uncomment if you are using raw values
  Serial.print("Array: ");
  Serial.println(position);
#endif
}

void sensorLaterais() {
  sensorDir = analogRead(QRE_RIGHT);
  sensorEsq = analogRead(QRE_LEFT);

  // Verifica se sensor direito detectou a linha
  if ((sensorDir < LEITURA_DIR) && sensorDirLinha) {
    sensorDirLinha = false;
    sensorDirCount++;
  } else if (sensorDir > LEITURA_DIR)
    sensorDirLinha = true;

  // Verifica se sensor esquerdo detectou a linha
  if ((sensorEsq < LEITURA_ESQ) && sensorEsqLinha) {
    sensorEsqLinha = false;
    sensorEsqCount++;
  } else if (sensorEsq > LEITURA_ESQ)
    sensorEsqLinha = true;

#if DEBUG_LTRAIS == 1
  Serial.print("Dir: ");
  Serial.print(sensorDir);
  Serial.print('\t');
  Serial.print("Esq: ");
  Serial.println(sensorEsq);

  Serial.print("countDir: ");
  Serial.print(sensorDirCount);
  Serial.print('\t');
  Serial.print("countEsq: ");
  Serial.println(sensorEsqCount);
  Serial.println("---------------");
#endif
}

int PID() {
  int error = 2500 - position;
  int p = error * KP;
  iPID = (iPID + error) * KI;
  int d = (error - lastError) * KD;
  lastError = error;
  return (p + iPID + d);
}

void pinConfiguration() {
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(QRE_LEFT, INPUT);
  pinMode(QRE_RIGHT, INPUT);
}

void velociadeCond() {
  //veloMax = (sensorEsqCount%2) > 0 ? MAX_SPEED : MAX_SPEED_LINE;
  //veloBase = (sensorEsqCount%2) > 0 ? BASE_SPEED : BASE_SPEED_LINE;

  // Impede o motor de ir alem da velocidade maxima
  if (rightMotorSpeed > veloMax)
    rightMotorSpeed = veloMax;
  if (leftMotorSpeed > veloMax)
    leftMotorSpeed = veloMax;

  // Mantem a velocidade do motor positiva
  if (rightMotorSpeed < 0)
    rightMotorSpeed = 5;
  if (leftMotorSpeed < 0)
    leftMotorSpeed = 5;
}

void velocidadeCalc() {
  rightMotorSpeed = veloBase + PID();
  leftMotorSpeed = veloBase - PID();
  //Serial.print("PID: ");
  //Serial.println(PIDcalc);
}

void velociadeControl() {
  digitalWrite(RIGHT_MOTOR, LOW);
  analogWrite(RIGHT_MOTOR_PWM, rightMotorSpeed);
  digitalWrite(LEFT_MOTOR, LOW);
  analogWrite(LEFT_MOTOR_PWM, leftMotorSpeed);

 /*  Serial.print(leftMotorSpeed);
  Serial.print('\t');
  Serial.print(rightMotorSpeed);
  Serial.println('\t'); */
}

void stopCondition() {
  // Caso stisfaça os pontos de parada, inicia parada dos motores
  if (sensorDirCount >= STOP_INDICATORS) {
    digitalWrite(RIGHT_MOTOR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, veloBase);
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, veloBase);
    delay(300);
    digitalWrite(RIGHT_MOTOR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, 0);
    delay(15000);
  }
}