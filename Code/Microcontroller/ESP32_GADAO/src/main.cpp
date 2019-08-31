#define WIFI_MODO STA

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>

AsyncWebServer server(80);

const char *ssid = "TT-Gadao";
const char *password = "guerreiro";

int indicator = 2;

bool LED = LOW;

void buttonClicked(const char *id)
{
    LED = !LED;
    digitalWrite(indicator, LED);
}

void setup()
{

    Serial.begin(9600);
    pinMode(indicator, OUTPUT);
     WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    ESPDash.init(server);

    ESPDash.addButtonCard("btn1", "Botão LED");
    ESPDash.attachButtonClick(buttonClicked);

    server.begin();
}

void loop()
{
}