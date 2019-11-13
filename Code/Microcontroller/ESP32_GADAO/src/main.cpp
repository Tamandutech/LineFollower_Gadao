#define WIFI_MODE_CONNECT

#include <Adafruit_MCP3008.h>
#include <Arduino.h>
#include <AsyncOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Hash.h>
#include <QTRSensors.h>
#include <WiFi.h>

#define NUM_SENSORS 6            // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
#define EMITTER_PIN 4            // emitter is controlled by digital pin 2

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]){36, 39, 34, 35, 32, 33}, NUM_SENSORS,
                      NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

AsyncWebServer server(80);
Adafruit_MCP3008 adc;

#if defined WIFI_MODE_SERVE
const char *ssid = "TT-Gadao";
const char *password = "guerreiro";
#elif defined WIFI_MODE_CONNECT
const char *ssid = "RFREITAS";
const char *password = "941138872";
#endif

int count = 0;

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT 13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN LED_BUILTIN

int brightness = 0; // how bright the LED i

int indicator = 2;

bool LED = LOW;

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void sliderAlterado(const char *id, const int sliderValue) {
  ledcAnalogWrite(LEDC_CHANNEL_0, sliderValue);
}

void setup() {

  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);

  Serial.begin(9600);

#if defined WIFI_MODE_SERVE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
#elif defined WIFI_MODE_CONNECT
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
#endif

  adc.begin();

  ESPDash.init(server);

  // ESPDash.addButtonCard("btn1", "Botão LED");
  // ESPDash.attachButtonClick(buttonClicked);

  ESPDash.addSliderCard("slider1", "Slider PWM", 2);
  ESPDash.attachSliderChanged(sliderAlterado);

  ESPDash.addNumberCard("num1", "Sensor 1", 0);
  ESPDash.addNumberCard("num2", "Sensor 2", 0);
  ESPDash.addNumberCard("num3", "Sensor 3", 0);
  ESPDash.addNumberCard("num4", "Sensor 4", 0);
  ESPDash.addNumberCard("num5", "Sensor 5", 0);
  ESPDash.addNumberCard("num6", "Sensor 6", 0);
  ESPDash.addNumberCard("num7", "Sensor 7", 0);
  ESPDash.addNumberCard("num8", "Sensor 8", 0);

  // OTA
  AsyncOTA.begin(&server); // Inicia o servidor de OTA

  server.begin();
  adc.begin(22);

  delay(500);
  pinMode(23, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  AsyncOTA.loop();
  ESPDash.updateNumberCard("num1", adc.readADC(0));
  delay(100);
  ESPDash.updateNumberCard("num2", adc.readADC(1));
  delay(100);
  ESPDash.updateNumberCard("num3", adc.readADC(2));
  delay(100);
  ESPDash.updateNumberCard("num4", adc.readADC(3));
  delay(100);
  ESPDash.updateNumberCard("num5", adc.readADC(4));
  delay(100);
  ESPDash.updateNumberCard("num6", adc.readADC(5));
  delay(100);
  ESPDash.updateNumberCard("num7", adc.readADC(6));
  delay(100);
  ESPDash.updateNumberCard("num8", adc.readADC(7));
  delay(100);
}
