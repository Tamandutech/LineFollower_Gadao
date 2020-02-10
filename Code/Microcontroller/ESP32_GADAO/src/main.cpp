#define WIFI_MODE_CONNECT

#include <Arduino.h>
#include <AsyncTCP.h>
#include <AutoPID.h>
#include <ESP32Encoder.h>
#include <ESP32MotorControl.h>
#include <ESP8266FtpServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Hash.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <io.h>
#include <mcp3008_linesensor.h>

double inputPID, setpointPID = 0.0, outputPID, outputMinPID = -100,
                 outputMaxPID = 100, Kp = 0.028, Ki = 0.0, Kd = 0.0;

AutoPID dirPID(&inputPID, &setpointPID, &outputPID, outputMinPID, outputMaxPID,
               Kp, Ki, Kd);

AsyncWebServer server(80);
TaskHandle_t DASH;
FtpServer ftpSrv;

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
const char *ssid = "RFREITAS";
const char *password = "941138872";
#endif

#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_13_BIT 13
#define LEDC_BASE_FREQ 5000
#define LED_PIN LED_BUILTIN

using namespace mcp3008;

bool DASHUpdateTimer = false;

int rightBaseSpeed = 0;
int leftBaseSpeed = 0;

void cb_timer() { DASHUpdateTimer = true; }

void startTimer() {
  // inicialização do timer.Parametros :
  /* 0 - seleção do timer a ser usado, de 0 a 3.
    80 - prescaler. O clock principal do ESP32 é 80MHz. Dividimos por 80
  para ter 1us por tick. true - true para contador progressivo, false para
  regressivo
  */
  timer = timerBegin(0, 80, true);

  /*conecta à interrupção do timer
   - timer é a instância do hw_timer
   - endereço da função a ser chamada pelo timer
   - edge=true gera uma interrupção
  */
  timerAttachInterrupt(timer, &cb_timer, true);

  /* - o timer instanciado no inicio
     - o valor em us para 1s
     - auto-reload. true para repetir o alarme
  */
  timerAlarmWrite(timer, 300000, true);

  // ativa o alarme
  timerAlarmEnable(timer);
}

void stopTimer() {
  timerEnd(timer);
  timer = NULL;
}

void DASHUpdate() {
  ESPDash.updateIRArrayCard(
      "encoders", new int[2]{enc_esq.getCount(), enc_dir.getCount()}, 2);
}

void sliderAlterado(const char *id, const int sliderValue) {
  rightBaseSpeed = sliderValue;
  leftBaseSpeed = sliderValue;
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
  MotorControl.attachMotors(MOT_ESQ_F, MOT_ESQ_R, MOT_DIR_F, MOT_DIR_R);

  Serial.begin(9600);

#if defined WIFI_MODE_SERVE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
#elif defined WIFI_MODE_CONNECT
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("ESP-BRAIA");
  WiFi.begin(ssid, password);
#endif

  // Realiza conexão do WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());

  SPIFFS.begin(true);
  ESPDash.init(server);
  ftpSrv.begin("esp32", "tamandutech");

  ESPDash.addSliderCard("motores", "Controle Motores", 0);
  ESPDash.attachSliderChanged(sliderAlterado);

  ESPDash.addIRArrayCard("encoders", "Encoders");

  server.begin();
  startTimer();

  Driver::Config cfg;
  cfg.pin_cs = GPIO_NUM_22;
  cfg.pin_miso = GPIO_NUM_19;
  cfg.pin_mosi = GPIO_NUM_23;
  cfg.pin_sck = GPIO_NUM_18;
  cfg.spi_dev = VSPI_HOST;

  ESP_ERROR_CHECK(ls.install(cfg));

  dirPID.setTimeStep(0);
}

void loop() {
  dirPID.run();

  if (DASHUpdateTimer) {
    DASHUpdate();
    DASHUpdateTimer = false;
  }
  ESPDash.loop();
  ftpSrv.handleFTP();

  for (int i = 0; i < Driver::CHANNELS; ++i) {
    Serial.print(ls.readChannel(i));
    Serial.print("\t");
  }

  Serial.println(ls.readLine(true));

  float erroLido = ls.readLine(true);

  if (erroLido >= -1 && erroLido <= 1)
    inputPID = (erroLido * 4000);

  Serial.printf("inputPID: %2.f\n", inputPID);
  Serial.printf("outputPID: %2.f\n", outputPID);

  int rightMotorSpeed = rightBaseSpeed + outputPID;
  int leftMotorSpeed = leftBaseSpeed - outputPID;

  if (rightMotorSpeed == 0)
    rightMotorSpeed = 1;

  if (leftMotorSpeed == 0)
    leftMotorSpeed = 1;

  MotorControl.motorForward(0, leftMotorSpeed);
  MotorControl.motorForward(1, rightMotorSpeed);
}
