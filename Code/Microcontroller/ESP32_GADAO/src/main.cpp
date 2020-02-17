#define WIFI_MODE_CONNECT
#define DEBUG_ON

#define HOST_NAME "ESP32Braia"

#include <Arduino.h>
#include <RemoteDebug.h>
//#include <AsyncTCP.h>
#include <AutoPID.h>
#include <ESP32Encoder.h>
#include <ESP32MotorControl.h>
#include <ESP8266FtpServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Hash.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <io.h>
#include <mcp3008_linesensor.h>

struct valuesPID {
  double input = 0.0;
  double setpoint = 0.0;
  double output = 0.0;
  double outputMin = 0.0;
  double outputMax = 0.0;
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;
};

struct valuesCar {
  float erroLido = 0.0;
  int rightBaseSpeed = 0;
  int leftBaseSpeed = 0;
  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;
  int gainKp = 0;
  int gainKd = 0;
};

valuesPID dirPIDVal;

valuesCar carVal;

AutoPID dirPID(&(dirPIDVal.input), &(dirPIDVal.setpoint), &(dirPIDVal.output),
               dirPIDVal.outputMin, dirPIDVal.outputMax, dirPIDVal.Kp,
               dirPIDVal.Ki, dirPIDVal.Kd);

RemoteDebug Debug;

AsyncWebServer server(80);

// Time
uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;

uint32_t lastTime = 0;

// Instancia do controlador dos motores
ESP32MotorControl MotorControl = ESP32MotorControl();

// Instancia dos encodes
ESP32Encoder enc_esq;
ESP32Encoder enc_dir;

// Instância do timer
hw_timer_t *timer = NULL;

// Instância do array
using namespace mcp3008;
LineSensor ls;

#if defined WIFI_MODE_SERVE
const char *ssid = "TT-Gadao";
const char *password = "guerreiro";
#elif defined WIFI_MODE_CONNECT
const char *ssid = "LineFollower_Gadao";
const char *password = "guerreiro";
#endif

using namespace mcp3008;

void sliderAlterado(const char *id, const int sliderValue) {
  if (String(id).equals("velmotpidmax")) {

    dirPID.setOutputRange(-sliderValue, sliderValue);
    dirPIDVal.outputMax = sliderValue;

    debugW("VelMotPID: %f\n", dirPIDVal.outputMax);

  } else if (String(id).equals("velmot")) {

    carVal.rightBaseSpeed = carVal.leftBaseSpeed = sliderValue;

    debugW("VelMot: %d\n", carVal.rightBaseSpeed);

  } else if (String(id).equals("Kp")) {

    carVal.gainKp = sliderValue;

    debugW("Kp calculado: %f\n", dirPIDVal.Kp);

  } else if (String(id).equals("Kd")) {
    carVal.gainKd = sliderValue;

    debugW("Kd calculado: %f\n", dirPIDVal.Kp);
  }

  dirPIDVal.Kp = ((dirPIDVal.outputMax + carVal.rightBaseSpeed) / 3500) *
                 (1 + (carVal.gainKp / 100));

  dirPIDVal.Kd = dirPIDVal.Kp * (1 + (carVal.gainKd / 100));

  dirPID.setGains(dirPIDVal.Kp, 0.0, dirPIDVal.Kd);
  dirPID.setGains(dirPIDVal.Kp, 0.0, dirPIDVal.Kd);
}

void setup() {
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

  // Kp, Ki, Kd
  // dirPID.setOutputRange(-15, 15);
  // dirPID.setGains(((dirPIDVal.outputMax + carVal.rightBaseSpeed) / 3500),
  // 0.0,
  //                 0.0);
  // debugW("Kp calculado: %f\n", dirPIDVal.Kp);

  // carVal.leftBaseSpeed = 30;
  // carVal.rightBaseSpeed = 30;

#if defined WIFI_MODE_SERVE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
#elif defined WIFI_MODE_CONNECT
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("ESP-BRAIA");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
    ;
#endif

  Debug.begin(HOST_NAME);
  Debug.setResetCmdEnabled(true);
  // Debug.showProfiler(true);
  Debug.showColors(true);

  SPIFFS.begin(true);
  ESPDash.init(server);

  ESPDash.addSliderCard("velmotpidmax", "Vel PID", 2);
  ESPDash.addSliderCard("velmot", "Vel", 2);
  ESPDash.addSliderCard("Kp", "Kp", 2);
  ESPDash.addSliderCard("Kd", "Kd", 2);

  ESPDash.attachSliderChanged(sliderAlterado);

  Driver::Config cfg;
  cfg.pin_cs = GPIO_NUM_22;
  cfg.pin_miso = GPIO_NUM_19;
  cfg.pin_mosi = GPIO_NUM_23;
  cfg.pin_sck = GPIO_NUM_18;
  cfg.spi_dev = VSPI_HOST;

  ESP_ERROR_CHECK(ls.install(cfg));

  dirPID.atSetPoint(0);
  dirPID.setTimeStep(1);
  server.begin();
}

void loop() {
  Debug.handle();
  ESPDash.loop();
  dirPID.run();

  carVal.erroLido = ls.readLine(true);

  if (carVal.erroLido >= -1 && carVal.erroLido <= 1)
    dirPIDVal.input = (carVal.erroLido * 3500);

  carVal.rightMotorSpeed = carVal.rightBaseSpeed + dirPIDVal.output;
  carVal.leftMotorSpeed = carVal.leftBaseSpeed - dirPIDVal.output;

  MotorControl.motorForward(0, constrain(carVal.leftMotorSpeed, 0, 100));
  MotorControl.motorForward(1, constrain(carVal.rightMotorSpeed, 0, 100));

  if ((lastTime + 1000) < millis()) {
    debugA("Leitura sensor: %f\n", carVal.erroLido);
    debugA("PIDin: %.f | PIDout: %.f\n", dirPIDVal.input, dirPIDVal.output);
    debugA("%d | %d | %d | %d | %d | %d | %d | %d\n", ls.readChannel(0),
           ls.readChannel(1), ls.readChannel(2), ls.readChannel(3),
           ls.readChannel(4), ls.readChannel(5), ls.readChannel(6),
           ls.readChannel(7));
    debugA("Motor esquerda: %d | Motor direita: %d\n", carVal.leftMotorSpeed,
           carVal.rightMotorSpeed);

    lastTime = millis();
  }
}