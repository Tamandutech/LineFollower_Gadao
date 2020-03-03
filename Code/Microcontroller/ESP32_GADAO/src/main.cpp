#pragma region *Defines

#define LINE LINE_BLACK
#define TRACK_LEFT_MARKS 57
#define TRACK_RIGHT_MARKS 999

#define LINE_WHITE < 1000
#define LINE_BLACK < 1000

#define TRACK_TIME 100

#pragma endregion Defines

#pragma region *Includes

#include <Adafruit_MCP3008.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Encoder.h>
#include <QTRSensors.h>
#include <SPIFFS.h>
#include <io.h>

#pragma endregion

#pragma region *Variables

String terminalBLE;

bool sLatEsq;
bool sLatDir;

unsigned long startTime, totalTime, trackTime = 99999999, serialTime;

struct valuesPID {
  double input = 0;
  double setpoint = 3500;
  double output = 0;
  double outputMin = -100;
  double outputMax = +100;
  double Kp = 0.0095;
  double Ki = 0.00;
  double Kd = 0.095;
};

struct valuesCar {
  int erroLido = 0;
  int rightBaseSpeed = 40;
  int leftBaseSpeed = 40;
  int maxSpeed = 100;
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
  int32_t encMedia = 0;
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

int markI = 0;

int acres = 180;

int enc_state[] = {
    // Curva (Inicio , Fim)
    993, 2163 + acres,
    // Curva
    3188, 3840 + acres,
    // Curva
    5514, 7227 + acres,
    // Curva
    7802, 8844 + acres,
    // Curva
    9262, 9796 + acres,
    // Curva
    10602, 11464 + acres - 50,
    // Curva
    11634, 12018 + acres,
    // Curva
    13917, 14644 + acres,
    // Curva
    16161, 17792 + acres,
    // Curva
    18388, 19371 + acres,
    // Curva
    19973, 20930 + acres,
    // Curva
    21851, 22544 + acres,
    // Curva
    22854, 23716 + acres,
    // Curva
    24228, 28410 + acres,
    // Fim
    30100 + acres};

int enc_stateI = 0;

#pragma endregion

#pragma region *Instances

valuesPID dirPIDVal;

valuesCar carVal;
valuesEnc motEncs;
valuesSLat sensorLat;

markTrack marksTrack[1000];

valuesLineSensor lsVal;

// Instancia dos encodes
ESP32Encoder enc_esq;
ESP32Encoder enc_dir;

// Instância do array
Adafruit_MCP3008 adc;
QTRSensors qtr;

#pragma endregion

#pragma region *Functions

#pragma region

void MotorControl(int motNumber, int pot, int dir) {

  switch (dir) {
  // Para frente
  case 0:
    switch (motNumber) {
    case 0:
      digitalWrite(MOT_ESQ_AI1, HIGH);
      digitalWrite(MOT_ESQ_AI2, LOW);
      ledcWrite(0, map(pot, 0, 100, 0,
                       1024)); // Escrevemos um duty cycle de 50% no canal 1.
      break;

    case 1:
      digitalWrite(MOT_DIR_BI1, HIGH);
      digitalWrite(MOT_DIR_BI2, LOW);
      ledcWrite(1, map(pot, 0, 100, 0,
                       1024)); // Escrevemos um duty cycle de 50% no canal 1.
      break;

    default:
      break;
    }
    break;

  // Para tras
  case 1:
    switch (motNumber) {
    case 0:
      digitalWrite(MOT_ESQ_AI1, LOW);
      digitalWrite(MOT_ESQ_AI2, HIGH);
      ledcWrite(0, map(pot, 0, 100, 0,
                       1024)); // Escrevemos um duty cycle de 50% no canal 1.
      break;

    case 1:
      digitalWrite(MOT_DIR_BI1, LOW);
      digitalWrite(MOT_DIR_BI2, HIGH);
      ledcWrite(1, map(pot, 0, 100, 0,
                       1024)); // Escrevemos um duty cycle de 50% no canal 1.
      break;

    default:
      break;
    }
    break;

  default:
    break;
  }
}

#pragma endregion

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
  motEncs.encMedia = (motEncs.encDir + motEncs.encEsq) / 2;
}

#pragma endregion

#pragma region **Funções do Terminal
void SerialSend() {

  Serial.printf("Setpoint: %.lf\n", dirPIDVal.setpoint);
  Serial.printf("Input: %.lf\n", dirPIDVal.input);
  Serial.printf("Output: %.lf\n", dirPIDVal.output);
  Serial.printf("---------\n");

  Serial.printf("LeftMotor: %d\n", carVal.leftMotorSpeed);
  Serial.printf("RightMotor: %d\n", carVal.rightMotorSpeed);
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
  Serial.printf("EncMedia: %d\n", motEncs.encMedia);
  Serial.printf("---------\n");

  Serial.printf("EncEsqCount: %d\n", carVal.leftMarksPassed);
  Serial.printf("EncDirCount: %d\n", carVal.rightMarksPassed);
  Serial.printf("---------\n");

  Serial.printf("Car status: %d\n", carVal.state);
  Serial.printf("---------\n");

  Serial.printf("Track time: %ld\n", trackTime);
  Serial.printf("---------\n");

  Serial.printf("Iterador: %ld\n", enc_stateI);
  Serial.printf("Iterador + 1: %ld\n", enc_stateI + 1);
  Serial.printf("---------\n");
}

#pragma endregion

#pragma region **Motor Control Func
void MotorControlFunc() {

  switch (carVal.state) {
  case 0: // Robô parado
    MotorControl(1, 0, 0);
    MotorControl(0, 0, 0);
    break;

  case 1: // Robô em linha

    carVal.rightMotorSpeed = 80 - dirPIDVal.output;
    carVal.leftMotorSpeed = 80 + dirPIDVal.output;

    MotorControl(
        0, constrain(carVal.leftMotorSpeed, carVal.minSpeed, carVal.maxSpeed),
        0);
    MotorControl(
        1, constrain(carVal.rightMotorSpeed, carVal.minSpeed, carVal.maxSpeed),
        0);

    break;

  case 2: // Robô em curva

    carVal.rightMotorSpeed = carVal.rightBaseSpeed - dirPIDVal.output;
    carVal.leftMotorSpeed = carVal.leftBaseSpeed + dirPIDVal.output;

    MotorControl(
        0, constrain(carVal.leftMotorSpeed, carVal.minSpeed, carVal.maxSpeed),
        0);
    MotorControl(
        1, constrain(carVal.rightMotorSpeed, carVal.minSpeed, carVal.maxSpeed),
        0);

    break;

  default:
    break;
  }
}

#pragma endregion

#pragma region **Processa a contagem dos sensores laterais
void processSLat() {

  // Verifica se sensores esquerdos e direitos não são acionados ao mesmo
  // tempo (caso de cuzamento)
  if (((sensorLat.sensorL1 < 1000 || sensorLat.sensorL2 < 1000)) &&
      !((sensorLat.sensorL3 < 1000 || sensorLat.sensorL4 < 1000)))
    sLatEsq = true;
  else if (sLatEsq == true)
    if (!((sensorLat.sensorL1 < 1000 || sensorLat.sensorL2 < 1000))) {
      sLatEsq = false;
      (carVal.leftMarksPassed)++;
    }

  // Verifica se sensores esquerdos e direitos não são acionados ao mesmo
  // tempo (caso de cuzamento)
  if (((sensorLat.sensorL3 < 1000 || sensorLat.sensorL4 < 1000)) &&
      !((sensorLat.sensorL1 < 1000 || sensorLat.sensorL2 < 1000)))
    sLatDir = true;
  else if (sLatDir == true)
    if (!((sensorLat.sensorL3 < 1000 || sensorLat.sensorL4 < 1000))) {
      sLatDir = false;
      (carVal.rightMarksPassed)++;
    }
}
#pragma endregion

#pragma region **Verifica estado do robo(curva, linha ou parada)

void verifyState() {
  // if (carVal.rightMarksPassed < TRACK_RIGHT_MARKS /* ||
  //     carVal.leftMarksPassed < TRACK_LEFT_MARKS */)
  //   if (carVal.leftMarksPassed % 2 != 0)
  //     carVal.state = 2;
  //   else
  //     carVal.state = 1;
  // else
  //   carVal.state = 0;

  if (carVal.state != 0) {
    int lastPos = (sizeof(enc_state) / sizeof(enc_state[0]));
    if (motEncs.encMedia > enc_state[lastPos -1]) {
      carVal.state = 0;
    } else {
      if (motEncs.encMedia >= enc_state[enc_stateI] &&
          motEncs.encMedia < enc_state[enc_stateI + 1]) {
        switch (carVal.state) {
        case 1:
          carVal.state = 2;
          enc_stateI++;
          break;

        case 2:
          carVal.state = 1;
          enc_stateI++;
          break;

        default:
          break;
        }
      }
    }
  }
}

#pragma endregion

#pragma region **PIDCalc

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

#pragma region **Imprime mapeamento da pista

void PrintMapping() {
  Serial.println("Tempo\tEncDir\tEncEsq\tMedia\tLatDir\n");

  for (int i = 0; i < markI; i++) {
    Serial.printf(
        "%ld\t%d\t%d\t%d\t%d\n", marksTrack[i].time,
        marksTrack[i].encVals.encDir, marksTrack[i].encVals.encEsq,
        (marksTrack[i].encVals.encDir + marksTrack[i].encVals.encEsq) / 2,
        marksTrack[i].slatVals.sensorL3);
  }
}

#pragma endregion

#pragma region **Faz o mapeamento da pista
void Trackapping() {
  marksTrack[markI].encVals = motEncs;
  marksTrack[markI].slatVals = sensorLat;
  marksTrack[markI].time = millis();

  trackTime = millis();
  markI++;
}
#pragma endregion

#pragma region *Setup

void setup() {
  // Define IO que controla o LED interno como saída
  pinMode(LED_BUILTIN, OUTPUT);

  // Define IOs que controlam o driver do motor como saída
  pinMode(MOT_ESQ_AI1, OUTPUT);
  pinMode(MOT_ESQ_AI2, OUTPUT);
  pinMode(MOT_ESQ_APWM, OUTPUT);
  pinMode(MOT_DIR_BI1, OUTPUT);
  pinMode(MOT_DIR_BI2, OUTPUT);
  pinMode(MOT_DIR_BPWM, OUTPUT);
  pinMode(MOTS_STANDBY, OUTPUT);

  digitalWrite(MOTS_STANDBY, HIGH);

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
  ledcAttachPin(MOT_ESQ_APWM, 0); // Atribuimos o pino 23 ao canal 0.
  ledcSetup(0, 1000, 10); // Atribuimos ao canal 0 a frequencia de 1000Hz com
                          // resolucao de 10bits.

  ledcAttachPin(MOT_DIR_BPWM, 1); // Atribuimos o pino 22 ao canal 1.
  ledcSetup(1, 1000, 10); // Atribuimos ao canal 1 a frequencia de 1000Hz com
                          // resolucao de 10bits.

  Serial.begin(115200);

  adc.begin(22);

  qtr.setTypeAnalog();
  qtr.setSensorPins(&adc, 8);

  SPIFFS.begin(true);

  if (digitalRead(0))
    calibSensor();
  else {
    MotorControl(0, 100, 0);
    MotorControl(1, 100, 0);
    carVal.state = 0;
  }
}

#pragma endregion

#pragma endregion

#pragma region *Loop

void loop() {
  if (carVal.state != 0) {
    while (1) {
      startTime = micros();

      GetData();
      PIDFollow();
      processSLat();
      verifyState();
      MotorControlFunc();

      if (sensorLat.sensorL3 < 1000 && trackTime == 99999999) {
        trackTime = 0;
        enc_dir.setCount(0);
        enc_esq.setCount(0);
      }

      if ((long)(millis() - trackTime) > TRACK_TIME)
        Trackapping();

      if (digitalRead(0) == LOW)
        carVal.state = 0;

      if (carVal.state == 0) {
        enc_dir.pauseCount();
        enc_esq.pauseCount();
        MotorControlFunc();
        while (1) {
          if (Serial.readString().equals("read"))
            PrintMapping();
        }
      }

      // Chama comandos do Terminal caso necessário
      if (millis() > serialTime) {
        SerialSend();
        serialTime += 500;
      }

      totalTime = micros() - startTime;
    }
  } else {
    while (1)
      ;
  }
}

#pragma endregion Loop