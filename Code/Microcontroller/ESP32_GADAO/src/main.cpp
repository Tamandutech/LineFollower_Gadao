#pragma region *Defines

#pragma endregion Defines

#pragma region *Includes

#include <Adafruit_MCP3008.h>
#include <Arduino.h>
#include <DabbleESP32.h>
#include <ESP32Encoder.h>
#include <ESP32MotorControl.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <QTRSensors.h>
#include <Ticker.h>
#include <io.h>

#pragma endregion

#pragma region *Variables

String terminalBLE = "";

byte ATuneModeRemember = 2;

double kpmodel = 1.5, taup = 100, theta[50];
double outputStart = 5;
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

boolean tuning = false;
unsigned long modelTime, serialTime;

// set to false to connect to the real world
boolean useSimulation = false;

struct valuesPID {
  double input = 0;
  double setpoint = 180;
  double output = 0;
  double outputMin = -47;
  double outputMax = +47;
  double Kp = 0.08;
  double Ki = 0.00;
  double Kd = 0.00;
};

struct valuesCar {
  int erroLido = 0;
  int rightBaseSpeed = 74;
  int leftBaseSpeed = 74;
  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;
  int gainKp = 0;
  int gainKd = 0;
  bool running = false;
};

struct valuesLineSensor {
  uint16_t channel[8];
  float line;
};

#pragma endregion

#pragma region *Instances

valuesPID dirPIDVal;

valuesCar carVal;

valuesLineSensor lsVal;

PID myPID(&(dirPIDVal.input), &(dirPIDVal.output), &(dirPIDVal.setpoint),
          dirPIDVal.Kp, dirPIDVal.Ki, dirPIDVal.Kd, DIRECT);

PID_ATune aTune(&(dirPIDVal.input), &(dirPIDVal.output));

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
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);
}
#pragma endregion

#pragma region **Funções do Tuning do PID

#pragma region ***AutoTuneHelper()
void AutoTuneHelper(boolean start) {
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}
#pragma endregion

#pragma region ***changeAutoTune()
void changeAutoTune() {
  if (!tuning) {
    // Set the output to the desired starting frequency.
    dirPIDVal.output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  } else { // cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}
#pragma endregion

#pragma region ***DoModel()
void DoModel() {
  // cycle the dead time
  for (byte i = 0; i < 49; i++) {
    theta[i] = theta[i + 1];
  }
  // compute the input
  dirPIDVal.input = (kpmodel / taup) * (theta[0] - outputStart) +
                    dirPIDVal.input * (1 - 1 / taup) +
                    ((float)random(-10, 10)) / 100;
}
#pragma endregion

#pragma region ***PIDmaster()
void PIDMaster() {
  unsigned long now = millis();

  if (!useSimulation) { // pull the input in from the real world
    dirPIDVal.input = qtr.readLineBlack(&(lsVal.channel[0]));
  }

  if (tuning) {
    byte val = (aTune.Runtime());
    if (val != 0) {
      tuning = false;
    }
    if (!tuning) { // we're done, set the tuning parameters
      dirPIDVal.Kp = aTune.GetKp();
      dirPIDVal.Ki = aTune.GetKi();
      dirPIDVal.Kd = aTune.GetKd();
      myPID.SetTunings(dirPIDVal.Kp, dirPIDVal.Ki, dirPIDVal.Kd);
      AutoTuneHelper(false);
    }
  } else
    myPID.Compute();

  if (useSimulation) {
    theta[30] = dirPIDVal.output;
    if (now >= modelTime) {
      modelTime += 100;
      DoModel();
    }
  } else {
    dirPIDVal.output = 0;
  }
}

#pragma endregion

#pragma endregion

#pragma region **Funções do Terminal
void SerialSend() {
  String terminalOUT = "";

  terminalOUT += "setpoint: ";
  terminalOUT += dirPIDVal.setpoint;
  terminalOUT += '\n';
  terminalOUT += "input: ";
  terminalOUT += dirPIDVal.input;
  terminalOUT += '\n';
  terminalOUT += "output: ";
  terminalOUT += dirPIDVal.output;
  terminalOUT += "\n---\n";
  terminalOUT += "velMot    velPID\n";
  terminalOUT += carVal.rightBaseSpeed;
  terminalOUT += "            ";
  terminalOUT += dirPIDVal.outputMax;
  terminalOUT += "\n---\n";
  if (tuning) {
    terminalOUT += "tuning mode";
  } else {
    terminalOUT += "Kp      Ki       Kd\n";
    terminalOUT += myPID.GetKp();
    terminalOUT += "   ";
    terminalOUT += myPID.GetKi();
    terminalOUT += "   ";
    terminalOUT += myPID.GetKd();
    terminalOUT += "   ";
  }

  Terminal.print(terminalOUT);
}

void SerialReceive() {
  if (Terminal.available() != 0) {
    terminalBLE = "";
    while (Terminal.available() != 0)
      terminalBLE += Terminal.read();

    switch (terminalBLE.charAt(0)) {
    case 't':
      if (terminalBLE.charAt(1) == 'u')
        changeAutoTune();
      break;

    case 'k':
      switch (terminalBLE.charAt(1)) {
      case 'p':
        dirPIDVal.Kp = terminalBLE.substring(2, terminalBLE.length()).toFloat();
        myPID.SetTunings(dirPIDVal.Kp, dirPIDVal.Ki, dirPIDVal.Kd);
        break;

      case 'i':
        dirPIDVal.Ki = terminalBLE.substring(2, terminalBLE.length()).toFloat();
        myPID.SetTunings(dirPIDVal.Kp, dirPIDVal.Ki, dirPIDVal.Kd);
        break;

      case 'd':
        dirPIDVal.Kd = terminalBLE.substring(2, terminalBLE.length()).toFloat();
        myPID.SetTunings(dirPIDVal.Kp, dirPIDVal.Ki, dirPIDVal.Kd);
        break;

      default:
        Terminal.print("Erro ao ler entrada\n");
        break;
      }

    case 'r':
      carVal.running = !carVal.running;
      break;

    case 'v':
      switch (terminalBLE.charAt(1)) {
      case 'p':
        dirPIDVal.outputMax =
            terminalBLE.substring(2, terminalBLE.length()).toInt();
        dirPIDVal.outputMin = -dirPIDVal.outputMax;
        myPID.SetOutputLimits(dirPIDVal.outputMin, dirPIDVal.outputMax);
        break;

      case 'm':
        carVal.leftBaseSpeed = carVal.rightBaseSpeed =
            terminalBLE.substring(2, terminalBLE.length()).toInt();
        break;

      default:
        break;
      }
      break;

    default:
      Terminal.println("Entrada inválida!");
      break;
    }
  }
}
#pragma endregion

#pragma region **Motor Control Func
void MotorControlFunc() {

  carVal.rightMotorSpeed = carVal.rightBaseSpeed + dirPIDVal.output;
  carVal.leftMotorSpeed = carVal.leftBaseSpeed - dirPIDVal.output;

  if (carVal.running) {
    MotorControl.motorForward(0, constrain(carVal.leftMotorSpeed, 0, 100));
    MotorControl.motorForward(1, constrain(carVal.rightMotorSpeed, 0, 100));
  } else {
    MotorControl.motorsStop();
  }
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
  Dabble.begin("Braia-BLE");

  adc.begin(22);

  qtr.setTypeAnalog();
  qtr.setSensorPins(&adc, 8);

  calibSensor();

  if (useSimulation) {
    for (byte i = 0; i < 50; i++) {
      theta[i] = outputStart;
    }
    modelTime = 0;
  }
  // Setup the pid
  myPID.SetMode(AUTOMATIC);

  if (tuning) {
    tuning = false;
    changeAutoTune();
    tuning = true;
  }
}

#pragma endregion

#pragma region *Loop

void loop() {
  
  Dabble.processInput();
  PIDMaster();
  MotorControlFunc();

  // Chama comandos do Terminal caso necessário
  if (millis() > serialTime) {
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
}

#pragma endregion Loop