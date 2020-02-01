#define WIFI_MODE_CONNECT

#include <Adafruit_MCP3008.h>
#include <Arduino.h>
//#include <AsyncOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Hash.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <ESP8266FtpServer.h>

AsyncWebServer server(80);
Adafruit_MCP3008 adc;

TaskHandle_t DASH;

FtpServer ftpSrv;

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



void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  ledcWrite(channel, duty);
}

void sliderAlterado(const char *id, const int sliderValue) {
  ledcAnalogWrite(LEDC_CHANNEL_0, sliderValue);
}

void DASHCode(void *parameter) {
  unsigned long timeDASH300 = 0;
  unsigned long timeDASH100 = 0;

  
  SPIFFS.begin(true);
  ESPDash.init(server);
  ftpSrv.begin("esp32","tamandutech"); 

  ESPDash.addSliderCard("slider1", "Slider PWM", 2);
  ESPDash.attachSliderChanged(sliderAlterado);
  ESPDash.addIRArrayCard("array1", "Array 1");

  // AsyncOTA.begin(&server);
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin",
                                       "http://localhost:8080");

  server.onNotFound([](AsyncWebServerRequest *request) {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      request->send(404);
    }
  });

  server.begin();

  for (;;) {
    ESPDash.loop();
    ftpSrv.handleFTP(); 

    if ((timeDASH300 + 300) < millis()) {
      ESPDash.updateIRArrayCard(
          "array1", new int[8]{random(0,1023), random(0,1023), random(0,1023),
                               random(0,1023), random(0,1023), random(0,1023),
                               random(0,1023), random(0,1023)});

      timeDASH300 = millis();
    }

    if ((timeDASH100 + 300) < millis()) {

      ESPDash.updateSliderCard("slider1", random(0,100));

      timeDASH100 = millis();
    }
  }
}

void setup() {
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

  adc.begin(22);

  pinMode(23, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  xTaskCreatePinnedToCore(DASHCode, /* Function to implement the task */
                          "DASH",   /* Name of the task */
                          10000,    /* Stack size in words */
                          NULL,     /* Task input parameter */
                          1,        /* Priority of the task */
                          &DASH,    /* Task handle. */
                          1);       /* Core where the task should run */
}

void loop() {}
