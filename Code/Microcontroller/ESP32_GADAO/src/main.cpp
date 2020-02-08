#define WIFI_MODE_CONNECT

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESP32Encoder.h>
#include <ESP32MotorControl.h>
#include <ESP8266FtpServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Hash.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <io.h>

AsyncWebServer server(80);
TaskHandle_t DASH;
FtpServer ftpSrv;

// Instancia do controlador dos motores
ESP32MotorControl MotorControl = ESP32MotorControl();

// Instancia dos encodes
ESP32Encoder enc_esq;
ESP32Encoder enc_dir;

TaskHandle_t TaskHandleDASHUpdate = NULL;

#if defined WIFI_MODE_SERVE
const char *ssid = "TT-Gadao";
const char *password = "guerreiro";
#elif defined WIFI_MODE_CONNECT
const char *ssid = "RFREITAS";
const char *password = "941138872";
#endif

#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_13_BIT 13
#define LEDC_BASE_FREQ 5000
#define LED_PIN LED_BUILTIN

void sliderAlterado(const char *id, const int sliderValue) {
  MotorControl.motorForward(0, sliderValue);
  MotorControl.motorForward(1, sliderValue);
}

static void DASHUpdate(void *pvParameters) {
  ESPDash.updateIRArrayCard(
      "encoders",
      new int[2]{(int32_t)enc_esq.getCount(), (int32_t)enc_dir.getCount()}, 2);
}

void setup() {
  // Define IO que controla LED da placa como saída
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
  MotorControl.attachMotor(MOT_ESQ_F, MOT_ESQ_R);
  MotorControl.attachMotor(MOT_DIR_F, MOT_DIR_R);

#if defined WIFI_MODE_SERVE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
#elif defined WIFI_MODE_CONNECT
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("ESP-BRAIA");
  WiFi.begin(ssid, password);
#endif

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  SPIFFS.begin(true);
  ESPDash.init(server);
  ftpSrv.begin("esp32", "tamandutech");

  ESPDash.addSliderCard("motores", "Controle Motores", 0);
  ESPDash.attachSliderChanged(sliderAlterado);

  ESPDash.addIRArrayCard("encoders", "Encoders");

  server.begin();
}

void loop() {
  ESPDash.loop();
  ftpSrv.handleFTP();
}
