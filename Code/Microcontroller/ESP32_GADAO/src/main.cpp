#define WIFI_MODE_CONNECT

#include <Arduino.h>
#include <Hash.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <WiFi.h>
#include <AsyncOTA.h>

AsyncWebServer server(80);

#if defined WIFI_MODE_SERVE
const char *ssid = "TT-Gadao";
const char *password = "guerreiro";
#elif defined WIFI_MODE_CONNECT
const char *ssid = "RFREITAS";
const char *password = "941138872";
#endif

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

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100)
{
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void buttonClicked(const char *id)
{
  LED = !LED;
  digitalWrite(indicator, LED);
}

void sliderAlterado(const char *id, const int sliderValue)
{
  ledcAnalogWrite(LEDC_CHANNEL_0, sliderValue);
}

void setup()
{

  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);

  Serial.begin(9600);
  pinMode(indicator, OUTPUT);

#if defined WIFI_MODE_SERVE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

#elif defined WIFI_MODE_CONNECT
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.printf("WiFi Failed!\n");
    return;
  }

#endif

  ESPDash.init(server);

  ESPDash.addButtonCard("btn1", "Botão LED");
  ESPDash.attachButtonClick(buttonClicked);

  ESPDash.addSliderCard("slider1", "Slider PID", 2);
  ESPDash.attachSliderChanged(sliderAlterado);

  //OTA
  AsyncOTA.begin(&server);    // Inicia o servidor de OTA

  server.begin();
}

void loop()
{
  // loop do OTA
  AsyncOTA.loop();
}