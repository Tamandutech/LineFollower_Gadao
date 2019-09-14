#include <Arduino.h>
#include <Atributos.h>

//biblioteca para o array
#include <QTRSensors.h>

//variaveis do array (valor de posicao sera recebida pela leitura do array)
unsigned int position = 0;
uint16_t sensors[NUM_SENSORS];

//variaveis para contagem e parada do robo
unsigned long previousMillis = 0;
int counter = 0;

//variaveis do PID
int lastError = 0;
int i = 0;

//variaveis para o controle de velocidade
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

//variaveis para o sensor da esquerda
int linhaPreta = true;
boolean reta = true;

//variaveis para o sensor da direita
int linhaPreta_Direita = true;
int aux = 0;
unsigned long timeInit = 0;

//pinos aos quais o array esta conectado, leitura digital (analogico tbm pode ler como digital)
QTRSensors qtra;

void calibrate_QTRSensors();
void monitor_SensoresLaterais();
void monitorSerial_calibrate();
void monitorSerial_QTRMeasurement();
int PID();
void pinConfiguration();
void speedCondition();
void speedControl();
void stopCondition();

void setup() {
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){1, 2, 3, 4, 5, 6}, NUM_SENSORS);
  qtra.setEmitterPin(EMITTER_PIN);
  qtra.setSamplesPerSensor(NUM_SAMPLES_PER_SENSOR);


  pinConfiguration();
  calibrate_QTRSensors();
  Serial.begin(9600);
  //monitorSerial_calibrate();
} 

void loop() {
  
  position = qtra.readLineWhite(sensors); //get calibrated readings along with the line position
  //monitorSerial_QTRMeasurement();
  monitor_SensoresLaterais();  
  rightMotorSpeed = BASE_SPEED + PID();
  leftMotorSpeed = BASE_SPEED - PID();
  speedCondition();
  speedControl();
  stopCondition();
}

void calibrate_QTRSensors() {
  
  unsigned long lastTime = 0;
  boolean ledStatus = HIGH;
  
  for(int i=0; i<400; i++)  {
    qtra.calibrate(); // make the calibration take about 10 seconds
    
    //faz o LED piscar durante a calibragem
    if(millis() >= lastTime){
      lastTime = millis() + TIME_LED;
      ledStatus = !ledStatus;
      digitalWrite(LED, ledStatus);  
    }    
  }
  digitalWrite(LED, HIGH);
  delay(2000);  //wait for 2s to position the bot before entering the main loop
}

void monitor_SensoresLaterais() {
  Serial.print("Dir ");
  Serial.print(analogRead(QRE_RIGHT));
  Serial.print(" Esq ");
  Serial.print(analogRead(QRE_LEFT));
  Serial.print(" Counter: ");
  Serial.println(counter);
}

void monitorSerial_calibrate() {
  //print the calibration minimum values measured when emitters were on
  for(int i=0; i<NUM_SENSORS; i++)  {
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  //print the maximum values
  for(int i=0; i<NUM_SENSORS; i++)  {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void monitorSerial_QTRMeasurement()  {
  for(unsigned char i=0; i<NUM_SENSORS; i++)  {
    Serial.print(sensors[i]);
    Serial.print('\t');
  }
  //Serial.println(); //uncomment if you are using raw values
  Serial.print("Array: ");
  Serial.println(position);
}

int PID()  {
  int error = 2500 - position;
  int p = error * KP;
  i = (i + error) * KI;
  int d = (error - lastError) * KD;
  lastError = error;
  return (p + i + d);
}

void pinConfiguration() {
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(QRE_LEFT, INPUT);
  pinMode(QRE_RIGHT, INPUT);
}

void speedCondition() {
  // prevent the motor from going beyond max speed
  
  if(rightMotorSpeed > MAX_SPEED) rightMotorSpeed = MAX_SPEED;
  if(leftMotorSpeed > MAX_SPEED) leftMotorSpeed = MAX_SPEED;

  // keep the motor speed positive
  if(rightMotorSpeed < 0) rightMotorSpeed = 5;
  if(leftMotorSpeed < 0)  leftMotorSpeed = 5;
}

void speedControl() {
  digitalWrite(RIGHT_MOTOR, LOW);
  analogWrite(RIGHT_MOTOR_PWM, rightMotorSpeed);
  digitalWrite(LEFT_MOTOR, LOW);
  analogWrite(LEFT_MOTOR_PWM, leftMotorSpeed);
}

void stopCondition()  {
  int stopSensorRead = analogRead(QRE_RIGHT);

  if(stopSensorRead < LEITURA_DIR && linhaPreta_Direita == true)
  {
    linhaPreta_Direita = false;
    counter++;
  }
  
  if(stopSensorRead > LEITURA_DIR)
  {
    linhaPreta_Direita = true;
  }
  
  if(counter >= STOP_INDICATORS) {
    digitalWrite(RIGHT_MOTOR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, BASE_SPEED);
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, BASE_SPEED);
    delay(300);
    digitalWrite(RIGHT_MOTOR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, 0);
    delay(15000);
  }

  /*
  unsigned long currentMillis = millis();
  stopSensorRead = analogRead(QRE_RIGHT);
  //Serial.println(stopSensorRead);

  if(stopSensorRead < 700 && (currentMillis - previousMillis >= interval))  {
    previousMillis = currentMillis;
    counter = counter + 1;
    //Serial.println(counter);
  
  }
  */
}