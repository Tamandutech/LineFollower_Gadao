#pragma region *Defines

#define LINE LINE_BLACK
#define TRACK_LEFT_MARKS 57
#define TRACK_RIGHT_MARKS 12

#define LINE_WHITE < 2000
#define LINE_BLACK < 2000

#pragma endregion Defines

#pragma region *Includes

#include <Adafruit_MCP3008.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <ESP32MotorControl.h>
#include <QTRSensors.h>
#include <io.h>

#pragma endregion

#pragma region *Variables

String terminalBLE;

bool sLatEsq;
bool sLatDir;

boolean tuning = false;
unsigned long startTime, totalTime, serialTime;

struct valuesPID {
  double input = 0;
  double setpoint = 3500;
  double output = 0;
  double outputMin = -100;
  double outputMax = +100;
  double Kp = 0.0093;
  double Ki = 0.00;
  double Kd = 0.093;
};

struct valuesCar {
  int erroLido = 0;
  int rightBaseSpeed = 19;
  int leftBaseSpeed = 19;
  int maxSpeed = 80;
  int minSpeed = 5;
  double rightMotorSpeed = 0;
  double leftMotorSpeed = 0;
  int gainKp = 0;
  int gainKd = 0;
  int leftMarksPassed = 0;
  int rightMarksPassed = 0;
  int state = 1; // 0: parado, 1: linha, 2: curva
};

struct valuesSLat {
  int sensorL1 = 0;
  int sensorL2 = 0;
  int sensorL3 = 0;
  int sensorL4 = 0;
};

struct valuesEnc {
  int32_t encDir = 0;
  int32_t encEsq = 0;
};

struct markTrack {
  valuesEnc encVals;
  valuesSLat slatVals;
  unsigned long time;
};

struct valuesLineSensor {
  uint16_t channel[8];
  float line;
};

// PID configs
float last_proportional = 0;
float proportional = 0;
float derivative = 0;
float integral = 0;

#pragma endregion

#pragma region *Instances

valuesPID dirPIDVal;

valuesCar carVal;
valuesEnc motEncs;
valuesSLat sensorLat;

markTrack marksTrack[TRACK_LEFT_MARKS];

valuesLineSensor lsVal;

// Instancia do controlador dos motores
ESP32MotorControl MotorControl = ESP32MotorControl();

// Instancia dos encodes
ESP32Encoder enc_esq;
ESP32Encoder enc_dir;

// Instância do array
Adafruit_MCP3008 adc;
QTRSensors qtr;

#pragma endregion

#pragma region *Functions

#pragma region **Calibrar Sensor Array
void calibSensor() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);
}
#pragma endregion

#pragma region **Update dos sensores

void GetData() {
  // Update erro do array
  dirPIDVal.input = qtr.readLineWhite(&(lsVal.channel[0]));

  // Update dos sensores laterais
  sensorLat.sensorL1 = analogRead(SL1);
  sensorLat.sensorL2 = analogRead(SL2);
  sensorLat.sensorL3 = analogRead(SL3);
  sensorLat.sensorL4 = analogRead(SL4);

  // Update dos encoders dos motores
  motEncs.encDir = enc_dir.getCount() * -1;
  motEncs.encEsq = enc_esq.getCount();
}

#pragma endregion

#pragma region **Funções do Terminal
void SerialSend() {

  Serial.printf("Setpoint: %.lf\n", dirPIDVal.setpoint);
  Serial.printf("Input: %.lf\n", dirPIDVal.input);
  Serial.printf("Output: %.lf\n", dirPIDVal.output);
  Serial.printf("---------\n");

  Serial.printf("LeftMotor: %d\n", MotorControl.getMotorSpeed(0));
  Serial.printf("RightMotor: %d\n", MotorControl.getMotorSpeed(1));
  Serial.printf("---------\n");

  Serial.printf("velMot\tvelPID\n");
  Serial.printf("%d\t%.lf\n", carVal.rightBaseSpeed, dirPIDVal.outputMax);
  Serial.printf("---------\n");

  Serial.printf("Kp\tKi\tKd\n");
  Serial.printf("%lf\t%lf\t%lf\n", dirPIDVal.Kp, dirPIDVal.Ki, dirPIDVal.Kd);
  Serial.printf("---------\n");

  Serial.printf("Ciclo: %ld\n", totalTime);
  Serial.printf("---------\n");

  Serial.printf("SL1: %d\n", sensorLat.sensorL1);
  Serial.printf("SL2: %d\n", sensorLat.sensorL2);
  Serial.printf("SL3: %d\n", sensorLat.sensorL3);
  Serial.printf("SL4: %d\n", sensorLat.sensorL4);
  Serial.printf("---------\n");

  Serial.printf("EncEsq: %d\n", motEncs.encEsq);
  Serial.printf("EncDir: %d\n", motEncs.encDir);
  Serial.printf("---------\n");

  Serial.printf("EncEsqCount: %d\n", carVal.leftMarksPassed);
  Serial.printf("EncDirCount: %d\n", carVal.rightMarksPassed);
  Serial.printf("---------\n");
}

#pragma endregion

#pragma region **Motor Control Func
void MotorControlFunc() {

  carVal.rightMotorSpeed = carVal.rightBaseSpeed - dirPIDVal.output;
  carVal.leftMotorSpeed = carVal.leftBaseSpeed + dirPIDVal.output;

  if (carVal.state != 0) {
    MotorControl.motorForward(
        0, constrain(carVal.leftMotorSpeed, carVal.minSpeed, carVal.maxSpeed));
    MotorControl.motorForward(
        1, constrain(carVal.rightMotorSpeed, carVal.minSpeed, carVal.maxSpeed));
  } else {
    delay(500);
    MotorControl.motorsStop();
  }
}

#pragma endregion

#pragma region **Processa a contagem dos sensores laterais
void processSLat() {

  // Verifica se sensores esquerdos e direitos não são acionados ao mesmo
  // tempo (caso de cuzamento)
  if (((sensorLat.sensorL1 < 2000 || sensorLat.sensorL2 < 2000)) &&
      !((sensorLat.sensorL3 < 2000 || sensorLat.sensorL4 < 2000)))
    sLatEsq = true;
  else if (sLatEsq == true)
    if (!((sensorLat.sensorL1 < 2000 || sensorLat.sensorL2 < 2000))) {
      sLatEsq = false;
      (carVal.leftMarksPassed)++;
    }

  // Verifica se sensores esquerdos e direitos não são acionados ao mesmo
  // tempo (caso de cuzamento)
  if (((sensorLat.sensorL3 < 2000  /*|| sensorLat.sensorL4 < 2000)) &&
      !((sensorLat.sensorL1 < 2000 || sensorLat.sensorL2 < 2000 */)))
    sLatDir = true;
  else if (sLatDir == true)
    if (!((sensorLat.sensorL3 < 2000 /*  || sensorLat.sensorL4 < 2000 */))) {
      sLatDir = false;
      (carVal.rightMarksPassed)++;
    }
}
#pragma endregion

#pragma region **Verifica estado do robo(curva, linha ou parada)

void verifyState() {
  if (carVal.rightMarksPassed < TRACK_RIGHT_MARKS /* ||
      carVal.leftMarksPassed < TRACK_LEFT_MARKS */)
    if (carVal.leftMarksPassed % 2 != 0)
      carVal.state = 2;
    else
      carVal.state = 1;
  else
    carVal.state = 0;
}

#pragma endregion

#pragma region PIDCalc

void PIDFollow() {

  float P = 0, I = 0, D = 0;

  // CALCULO DE VALOR LIDO
  proportional = dirPIDVal.input - 3500;
  derivative = proportional - last_proportional;
  integral = integral + proportional;
  last_proportional = proportional;

  P = proportional * dirPIDVal.Kp;
  D = derivative * dirPIDVal.Kd;
  I = integral * dirPIDVal.Ki;
  // PID
  dirPIDVal.output = P + I + D;
}

#pragma endregion

#pragma endregion

#pragma region *Setup

void setup() {
  // Define IO que controla o LED interno como saída
  pinMode(LED_BUILTIN, OUTPUT);

  // Define IOs que controlam o driver do motor como saída
  pinMode(MOT_DIR_F, OUTPUT);
  pinMode(MOT_DIR_R, OUTPUT);
  pinMode(MOT_ESQ_F, OUTPUT);
  pinMode(MOT_ESQ_R, OUTPUT);

  // Define IOs que leem os sensores laterais como entrada
  pinMode(SL1, INPUT);
  pinMode(SL2, INPUT);
  pinMode(SL3, INPUT);
  pinMode(SL4, INPUT);

  // Define IOs que leem os ecnoders dos motores como entrada
  pinMode(ENC_MOT_DIR_A, INPUT);
  pinMode(ENC_MOT_DIR_B, INPUT);
  pinMode(ENC_MOT_ESQ_A, INPUT);
  pinMode(ENC_MOT_ESQ_B, INPUT);

  // Habilita os resistores internos de pulldown fraco
  ESP32Encoder::useInternalWeakPullResistors = true;

  // Anexa os IOs a instancia do contador dos encoders
  enc_dir.attachHalfQuad(ENC_MOT_DIR_A, ENC_MOT_DIR_B);
  enc_esq.attachHalfQuad(ENC_MOT_ESQ_A, ENC_MOT_ESQ_B);

  // Anexa os IOs dos motores ao controlador dos motores
  MotorControl.attachMotors(MOT_ESQ_F, MOT_ESQ_R, MOT_DIR_F, MOT_DIR_R);

  Serial.begin(115200);

  adc.begin(22);

  qtr.setTypeAnalog();
  qtr.setSensorPins(&adc, 8);

  calibSensor();
}

#pragma endregion

#pragma region *Loop

void loop() {
  startTime = micros();

  GetData();

  PIDFollow();

  processSLat();

  verifyState();

  MotorControlFunc();

  // Chama comandos do Terminal caso necessário
  if (millis() > serialTime) {
    SerialSend();
    serialTime += 500;
  }

  totalTime = micros() - startTime;
}

#pragma endregion Loop