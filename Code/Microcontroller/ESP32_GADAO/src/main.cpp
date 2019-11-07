#define WIFI_MODE_CONNECT

#include <Arduino.h>
#include <Hash.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <WiFi.h>
#include <AsyncOTA.h>
#include <QTRSensors.h>

#define NUM_SENSORS 6            // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
#define EMITTER_PIN 4            // emitter is controlled by digital pin 2

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]){36, 39, 34, 35, 32, 33},
                      NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

AsyncWebServer server(80);

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
#define LED_PIN 2

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

void IRAM_ATTR isr()
{
  count++;
}

void setup()
{

  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);

  pinMode(21, INPUT_PULLUP);
  attachInterrupt(21, isr, FALLING);

  pinMode(19, INPUT_PULLUP);
  attachInterrupt(19, isr, FALLING);

  Serial.begin(9600);

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

  //ESPDash.addButtonCard("btn1", "Botão LED");
  //ESPDash.attachButtonClick(buttonClicked);

  ESPDash.addSliderCard("slider1", "Slider PWM", 2);

  ESPDash.attachSliderChanged(sliderAlterado);

  //ESPDash.addNumberCard("num1", "Encoder 1", 0);

  //OTA
  AsyncOTA.begin(&server); // Inicia o servidor de OTA

  server.begin();

  delay(500);
  pinMode(23, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 2000; i++)   // make the calibration take about 10 seconds
  {
    qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  Serial.println("Terminou o setup!");
}

void loop()
{

  // loop do OTA
  AsyncOTA.loop();

  /*  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtra.read(sensorValues); instead of unsigned int position = qtra.readLine(sensorValues);
  unsigned int position = qtra.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    
    Serial.print('\t');
  }

  Serial.print(digitalRead(23));

  Serial.print('\t');

  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values

  //delay(250);
 */
  ESPDash.updateNumberCard("num1", count);
  delay(250);
}
/* 
#include <Arduino.h>
#define sensor 4

int i = 0;

void setup() {
  pinMode(sensor, INPUT);
  Serial.begin(9600);
}

void loop() {
  i++;
  int s1 = digitalRead(sensor);
  Serial.printf("Leitura %d: ", i);
  Serial.println(s1);
} */