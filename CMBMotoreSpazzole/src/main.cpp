#include <Arduino.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#include "headers.h"

DynamicJsonDocument data(1024);

// Simplefoc components
PIDController controllo_vel = PIDController(0.5, 10, 0.0, 100, 100);
Encoder encoder = Encoder(H1, H2, 88);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
LowPassFilter filter = LowPassFilter(0.5);

HardwareSerial logSerial = Serial;

TaskHandle_t TaskHandleControl;
TaskHandle_t TaskHandleData;
TaskHandle_t TaskHandleSerial;

void TaskControl(void *pvParameters);
void TaskData(void *pvParameters);
void TaskSerial(void *pvParameters);
int radSecToRpm(float radSec);
float rpmToRadSec(int rpm);

uint8_t currentSystemState = STATE_START;
uint8_t prevState = STATE_START;

float currentSpeed = 0;
float currentAngle = 0;
int currentPulses = 0;

int minPulses = 0;
int maxPulses = 0;
int railLengthPulses = 0;

long timeoutStart = 0;
bool pulsantePremuto = false;
long lastHallInterrupt = 0;
short potenziometro = 0;
float targetSpeed = 0;

float power = 0;

bool buttonPressed = false;
bool okReceived = false;

int currentMeasured = 0;

// parametri controllabili
long timeoutDuration = DEFAULT_TIMEOUT;
int pulseStart = DEFAULT_RAIL_LENGTH_PULSES * DEFAULT_RAIL_START;
int pulseEnd = DEFAULT_RAIL_LENGTH_PULSES * DEFAULT_RAIL_END;
int rpmOpen = radSecToRpm(DEFAULT_RAD_OPEN);
int rpmClose = radSecToRpm(DEFAULT_RAD_CLOSE);
float railStart = DEFAULT_RAIL_START;
float railEnd = DEFAULT_RAIL_END;

void initPins()
{
  pinMode(I_MOT, INPUT);

  // analogReadResolution(10);
  // analogSetAttenuation(ADC_0db);

  pinMode(PULSANTE, INPUT_PULLUP);
  pinMode(POTENZIOMETRO, INPUT_PULLUP);

  pinMode(H1, INPUT_PULLUP);
  pinMode(H2, INPUT_PULLUP);

  pinMode(ADC_BATT, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN1, PWM_CHANNEL_1);
  ledcAttachPin(IN2, PWM_CHANNEL_2);
}

void initSimpleFOC()
{

  // encoder
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_INTERN;

  encoder.init();
  encoder.enableInterrupts(doA, doB);
}

int radSecToRpm(float radSec)
{
  return radSec * 60 / (2 * PI);
}

float rpmToRadSec(int rpm)
{
  return rpm * 2 * PI / 60;
}

void setup()
{
  WiFi.mode(WIFI_OFF);

  Serial.begin(LOG_BAUD);

  logSerial.begin(LOG_BAUD, SERIAL_8N1);
  logSerial.println("Starting");

  initPins();

  initSimpleFOC();

  delay(50);

  currentSystemState = STATE_INACTIVE;

  xTaskCreatePinnedToCore(
      TaskControl,
      "TaskControl",
      5000,
      NULL,
      9,
      &TaskHandleControl,
      0);

  xTaskCreatePinnedToCore(
      TaskData,
      "TaskData",
      5000,
      NULL,
      9,
      &TaskHandleData,
      1);

  xTaskCreatePinnedToCore(
      TaskSerial,
      "Taskserial",
      5000,
      NULL,
      9,
      &TaskHandleSerial,
      1);
}

void loop()
{
  vTaskDelete(NULL);
}

bool updateState(int pulses, float speed, long millis)
{
  switch (currentSystemState)
  {
  case STATE_START:
    // dopo setup passo in automatico a inactive
    break;
  case STATE_INACTIVE:
    if (buttonPressed || okReceived)
    {
      buttonPressed = false;
      okReceived = false;

      railLengthPulses = (maxPulses - minPulses);
      pulseStart = minPulses + (railLengthPulses * railStart);
      pulseEnd = minPulses + (railLengthPulses * railEnd);

      currentSystemState = STATE_INIZIOCORSA;
    }
    break;
  case STATE_INIZIOCORSA:
    if (millis - timeoutStart > timeoutDuration)
    {
      timeoutStart = 0;
      currentSystemState = STATE_APERTURA;
    }
    break;
  case STATE_APERTURA:
    if (pulses >= pulseEnd)
    {
      timeoutStart = millis;
      currentSystemState = STATE_FINECORSA;
    }
    break;
  case STATE_FINECORSA:
    if (millis - timeoutStart > timeoutDuration)
    {
      timeoutStart = 0;
      currentSystemState = STATE_CHIUSURA;
    }
    break;
  case STATE_CHIUSURA:
    if (pulses <= pulseStart)
    {
      timeoutStart = millis;
      currentSystemState = STATE_INIZIOCORSA;
    }
    break;
  }
  return true;
}

void TaskControl(void *pvParameters) // task controllo motore
{
  int pwmPower = 0;

  while (1)
  {
    encoder.update();

    currentSpeed = encoder.getVelocity();
    currentSpeed = filter(currentSpeed);

    currentAngle = encoder.getAngle();
    currentPulses = (int)encoder.getAngle() * POLES;

    updateState(currentPulses, currentSpeed, millis());

    switch (currentSystemState)
    {
    case STATE_START:
      // non fare niente
      break;
    case STATE_INACTIVE:
      // motore staccato, salva minimo e massimo rotaia trovati

      minPulses = min(minPulses, currentPulses);
      maxPulses = max(maxPulses, currentPulses);

      ledcWrite(PWM_CHANNEL_1, 0);
      ledcWrite(PWM_CHANNEL_2, 0);
      break;
    case STATE_INIZIOCORSA:
      // freno attivo, non dare potenza
      ledcWrite(PWM_CHANNEL_1, 255);
      ledcWrite(PWM_CHANNEL_2, 255);
      break;
    case STATE_APERTURA:
      // muovi a velocità costante
      targetSpeed = rpmToRadSec(rpmOpen);
      break;
    case STATE_FINECORSA:
      // freno attivo, non dare potenza
      ledcWrite(PWM_CHANNEL_1, 255);
      ledcWrite(PWM_CHANNEL_2, 255);
      break;
    case STATE_CHIUSURA:
      // muovi a velocità costante inversa e maggiore
      targetSpeed = -rpmToRadSec(rpmClose);
      break;
    }

    power = controllo_vel(targetSpeed - currentSpeed);

    // if (pulsantePremuto || true)
    //{
    if (targetSpeed != 0)
    {
      if (power > 0)
      {
        ledcWrite(PWM_CHANNEL_1, 155 + power);
        ledcWrite(PWM_CHANNEL_2, 0);
      }
      else
      {
        ledcWrite(PWM_CHANNEL_1, 0);
        ledcWrite(PWM_CHANNEL_2, 155 - power);
      }
    }

    //}
    /*else
    {
      ledcWrite(PWM_CHANNEL_1, 0);
      ledcWrite(PWM_CHANNEL_2, 0);
    }*/

    delay(1);
  }
}

void TaskData(void *pvParameters) // task raccolta dati
{
  long lastMillis = 0;
  while (1)
  {
    if (millis() - lastMillis >= 10)
    {
      pulsantePremuto = !digitalRead(PULSANTE);
      if (pulsantePremuto)
      { // evento pressione pulsante richiesto
        buttonPressed = true;
      }
      potenziometro = analogRead(POTENZIOMETRO);

      currentMeasured = analogRead(I_MOT);//TODO trasformare in corrente
    }

    vTaskDelay(1);
  }
}

float getPercentagePosition()
{
  return (currentPulses - minPulses) / (maxPulses - minPulses);
}

void TaskSerial(void *pvParameters) // task comunicazione con seriale
{

  int lastSent = 0;
  while (1)
  {
    if (millis() - lastSent >= 10)
    {

      data["pulses"] = currentPulses;
      // data["angle"] = currentAngle;
      //  data["pulsante"] = pulsantePremuto;
      //  data["pwm"] = map(potenziometro, 0, 2000, 0, 255);
      data["posizione"] = getPercentagePosition();
      data["pwm"] = power;
      data["current"] = currentMeasured;
      data["target"] = radSecToRpm(targetSpeed);
      data["speed"] = radSecToRpm(currentSpeed);
      data["millis"] = millis();

      serializeJson(data, logSerial);
      logSerial.println();
      lastSent = millis();
    }

    if (logSerial.available())
    {
      String command = logSerial.readStringUntil('\n');

      if (command.indexOf("reset") >= 0)
      {
        ESP.restart();
      }

      if (command.indexOf("start") >= 0)
      {
        okReceived = true;
        logSerial.println("startOk");
      }

      logSerial.println("Command received: " + command);

      // check if string contains Set
      if (command.indexOf("Set") < 0)
      {
        continue;
      }

      long tmptimeoutDuration = UNDEFINED_VALUE;
      int tmppulseStart = UNDEFINED_VALUE;
      int tmppulseEnd = UNDEFINED_VALUE;
      int tmprpmOpen = UNDEFINED_VALUE;
      int tmprpmClose = UNDEFINED_VALUE;
      float tmprailStart = UNDEFINED_VALUE;
      float tmprailEnd = UNDEFINED_VALUE;

      sscanf(command.c_str(), "Set;%d;%d;%d;%d;%d;%f;%f;", &tmptimeoutDuration, &tmppulseStart, &tmppulseEnd, &tmprpmOpen, &tmprpmClose, &tmprailStart, &tmprailEnd);

      // TODO check input

      if (tmptimeoutDuration != UNDEFINED_VALUE)
      {
        timeoutDuration = tmptimeoutDuration;
      }
      if (tmppulseStart != UNDEFINED_VALUE)
      {
        pulseStart = tmppulseStart;
      }
      if (tmppulseEnd != UNDEFINED_VALUE)
      {
        pulseEnd = tmppulseEnd;
      }
      if (tmprpmOpen != UNDEFINED_VALUE)
      {
        rpmOpen = tmprpmOpen;
      }
      if (tmprpmClose != UNDEFINED_VALUE)
      {
        rpmClose = tmprpmClose;
      }
      if (tmprailStart != UNDEFINED_VALUE)
      {
        railStart = tmprailStart;
      }
      if (tmprailEnd != UNDEFINED_VALUE)
      {
        railEnd = tmprailEnd;
      }

      logSerial.println("SetOk");
    }
  }
}
