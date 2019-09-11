#define WIFI_MODE_CONNECT

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <WiFi.h>

AsyncWebServer server(80);

#if defined WIFI_MODE_SERVE
const char *ssid = "TT-Gadao";
const char *password = "guerreiro";
#elif defined WIFI_MODE_CONNECT
const char *ssid = "VIVOFIBRA-737C";
const char *password = "941138872";
#endif

int indicator = 2;

bool LED = LOW;

void buttonClicked(const char *id) {
  LED = !LED;
  digitalWrite(indicator, LED);
}

void setup() {

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
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }

#endif

  ESPDash.init(server);

  ESPDash.addButtonCard("btn1", "Botão LED");
  ESPDash.attachButtonClick(buttonClicked);

  server.begin();
}

void loop() {}