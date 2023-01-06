#include <Arduino.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <Preferences.h>

#include "headers.h"

void TaskControl(void *pvParameters);
void TaskData(void *pvParameters);
void TaskSerial(void *pvParameters);
int radSecToRpm(float radSec);
float rpmToRadSec(int rpm);

Preferences preferences;

// parametri controllo algoritmo
long timeoutDuration = DEFAULT_TIMEOUT;
long timeoutOpen = DEFAULT_OPEN_TIMEOUT;
int pulseStart = DEFAULT_RAIL_LENGTH_PULSES * DEFAULT_RAIL_START;
int pulseEnd = DEFAULT_RAIL_LENGTH_PULSES * DEFAULT_RAIL_END;
int rpmOpen = radSecToRpm(DEFAULT_RAD_OPEN);
int rpmClose = radSecToRpm(DEFAULT_RAD_CLOSE);
float railStart = DEFAULT_RAIL_START;
float railEnd = DEFAULT_RAIL_END;

// parametri pid apertura
float kpOpen = DEFAULT_KP_OPEN;
float kiOpen = DEFAULT_KI_OPEN;
float kdOpen = DEFAULT_KD_OPEN;

// parametri pid chiusura
float kpClose = DEFAULT_KP_CLOSE;
float kiClose = DEFAULT_KI_CLOSE;
float kdClose = DEFAULT_KD_CLOSE;

// Simplefoc components
PIDController controllo_vel_apre = PIDController(kpOpen, kiOpen, kdOpen, 300, 100); // configurazione originale
PIDController controllo_vel_chiude = PIDController(kpClose, kiClose, kdClose, 300, 100);
Encoder encoder = Encoder(H1, H2, 88);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
LowPassFilter speedFilter = LowPassFilter(0.5);
LowPassFilter voltageFilter = LowPassFilter(0.5);
LowPassFilter currentFilter = LowPassFilter(0.5);

HardwareSerial logSerial = Serial;

TaskHandle_t TaskHandleControl;
TaskHandle_t TaskHandleData;
TaskHandle_t TaskHandleSerial;

uint8_t currentSystemState = STATE_START;
uint8_t prevState = STATE_START;

float currentSpeed = 0;
float currentAngle = 0;
int currentPulses = 0;

int minPulses = 0;
int maxPulses = 0;
int railLengthPulses = 0;

long timeoutStart = 0;
long lastHallInterrupt = 0;
int potenziometro = 0;
float targetSpeed = 0;

float power = 0;

// input utente
bool buttonPressed = false;
bool buttonLongPressed = false;
bool okReceived = false;
bool stopReceived = false;
bool startConfig = false;
bool endConfig = false;

bool continousTestActive = false;

int currentMeasured = 0;
int batteryVoltage = 0;

void initPins()
{
  // pinMode(I_MOT, INPUT);

  // analogReadResolution(10);
  // analogSetAttenuation(ADC_0db);

  pinMode(PULSANTE, INPUT_PULLUP);
  // pinMode(POTENZIOMETRO, INPUT_PULLUP);

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

  // Serial.begin(LOG_BAUD);

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
      10,
      &TaskHandleControl,
      0);

  /*xTaskCreatePinnedToCore(
      TaskData,
      "TaskData",
      5000,
      NULL,
      9,
      &TaskHandleData,
      1);*/

  xTaskCreatePinnedToCore(
      TaskSerial,
      "Taskserial",
      5000,
      NULL,
      ESP_TASK_PRIO_MAX - 1,
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
  case STATE_INACTIVE:               // state 1
    if (buttonPressed || okReceived) // evento d'inizio ricevuto
    {
      buttonPressed = false;
      okReceived = false;

      preferences.begin(CONFIG_NAMESPACE);
      railLengthPulses = preferences.getInt("rail", DEFAULT_RAIL_LENGTH_PULSES);
      preferences.end();

      if (railLengthPulses < 50)
      {
        railLengthPulses = DEFAULT_RAIL_LENGTH_PULSES;
      }

      // railLengthPulses = (maxPulses - minPulses);
      pulseStart = minPulses + (railLengthPulses * railStart);
      pulseEnd = minPulses + (railLengthPulses * railEnd);

      timeoutStart = millis;
      currentSystemState = STATE_INIZIOCORSA;
    }
    else if (buttonLongPressed || startConfig)
    {
      buttonLongPressed = false;
      startConfig = false;

      currentSystemState = STATE_CONFIGURAZIONE;
    }
    else if (continousTestActive)
    {
      timeoutStart = millis;
      currentSystemState = STATE_INIZIOCORSA;
    }
    break;
  case STATE_INIZIOCORSA: // state 2
    /*if (buttonPressed || stopReceived)
    {
      buttonPressed = false;
      stopReceived = false;

      currentSystemState = STATE_INACTIVE;
      timeoutStart = 0;
    }
    else*/
    if (millis - timeoutStart > timeoutDuration && timeoutStart != 0)
    {
      timeoutStart = 0;
      currentSystemState = STATE_APERTURA;
      currentPulses = 0;
    }
    break;
  case STATE_APERTURA: // state 3
    if (pulses >= pulseEnd)
    {
      timeoutStart = millis;
      currentSystemState = STATE_FINECORSA;
    }
    break;
  case STATE_FINECORSA: // state 4
    if (millis - timeoutStart > timeoutOpen)
    {
      timeoutStart = 0;
      currentSystemState = STATE_CHIUSURA;
    }
    break;
  case STATE_CHIUSURA: // state 5
    if (pulses <= pulseStart)
    {
      // timeoutStart = millis;
      currentSystemState = STATE_INACTIVE;
    }
    break;
  case STATE_CONFIGURAZIONE: // state 6
    if (buttonLongPressed || endConfig)
    {
      buttonLongPressed = false;
      endConfig = false;

      preferences.begin(CONFIG_NAMESPACE);
      preferences.putInt("rail", maxPulses - minPulses);
      preferences.end();

      railLengthPulses = maxPulses - minPulses;

      logSerial.println("Configurazione completata, lunghezza rotaia: " + String(railLengthPulses));

      pulseStart = minPulses + (railLengthPulses * railStart);
      pulseEnd = minPulses + (railLengthPulses * railEnd);

      currentSystemState = STATE_INACTIVE;
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
    currentSpeed = speedFilter(currentSpeed);

    currentAngle = encoder.getAngle();
    currentPulses = (int)(encoder.getAngle() * POLES);

    updateState(currentPulses, currentSpeed, millis());

    switch (currentSystemState)
    {
    case STATE_START:
      // non fare niente
      break;
    case STATE_INACTIVE: // 1
      // motore staccato, attendi comando per inizio

      // minPulses = min(minPulses, currentPulses);
      // maxPulses = max(maxPulses, currentPulses);
      targetSpeed = 0;

      ledcWrite(PWM_CHANNEL_1, 0);
      ledcWrite(PWM_CHANNEL_2, 0);
      break;
    case STATE_INIZIOCORSA: // 2
      // freno attivo, non dare potenza
      targetSpeed = 0;
      ledcWrite(PWM_CHANNEL_1, 255);
      ledcWrite(PWM_CHANNEL_2, 255);
      break;
    case STATE_APERTURA: // 3
      // muovi a velocita'  costante
      targetSpeed = rpmToRadSec(rpmOpen);
      break;
    case STATE_FINECORSA: // 4
      // freno disaattivo, non dare potenza
      targetSpeed = 0;
      ledcWrite(PWM_CHANNEL_1, 0);
      ledcWrite(PWM_CHANNEL_2, 0);
      break;
    case STATE_CHIUSURA: // 5
      // muovi a velocita'  costante inversa e maggiore
      targetSpeed = rpmToRadSec(rpmClose);
      break;
    case STATE_CONFIGURAZIONE:
      // motore staccato per consentire movimento manuale, salva minimo e massimo valori rotaia
      minPulses = min(minPulses, currentPulses);
      maxPulses = max(maxPulses, currentPulses);
      targetSpeed = 0;

      ledcWrite(PWM_CHANNEL_1, 0);
      ledcWrite(PWM_CHANNEL_2, 0);
      break;
    }

    if (currentSystemState != STATE_INACTIVE && currentSystemState != STATE_CONFIGURAZIONE)
    {
      power = controllo_vel_apre(targetSpeed - currentSpeed);

      // if (pulsantePremuto || true)
      //{
      if (targetSpeed != 0)
      {

        if (currentSystemState == STATE_CHIUSURA)
        {
          power = controllo_vel_chiude(targetSpeed - currentSpeed);
          // float powerReverse = map(power, 0, 100, 100, 255);

          if (power > 0)
          {
            // Serial.println("we are NOT braking  ");
            ledcWrite(PWM_CHANNEL_1, map(power, 0, 100, 100, 255));
            ledcWrite(PWM_CHANNEL_2, 255);
          }
          else
          {
            // Serial.print("we are braking  ");
            // Serial.println();
            ledcWrite(PWM_CHANNEL_1, map(-power * 2.55, 0, 255, 255, 0));
            ledcWrite(PWM_CHANNEL_2, 255);
            // ledcWrite(PWM_CHANNEL_2, map(-power * 2.55, 0, 255, 255, 0));
          }
        }
        else
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

/*
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

      currentMeasured = analogRead(I_MOT);
      batteryVoltage = analogRead(ADC_BATT);
    }

    vTaskDelay(1);
  }
}*/

float getPercentagePosition()
{
  if (maxPulses == minPulses)
  {
    return 0;
  }
  else
  {
    return (currentPulses - minPulses) / (maxPulses - minPulses);
  }
}

int adcCurrentToMilliAmp(int adcValue)
{
  return map(adcValue, 640, 3300, -5000, 5000);
}

int adcVoltageToBatteryVoltage(int adcValue)
{
  return map(adcValue, 540, 846, 18000, 26000);
}

void TaskSerial(void *pvParameters) // task comunicazione con seriale
{

  long lastSent = 0;
  long buttonPressStart = 0;
  bool pulsantePremuto = false;

  int logState = 0;
  int logPulses = 0;
  float logPosizione = 0;
  float logPwm = 0;
  int logCurrent = 0;
  int logTarget = 0;
  int logSpeed = 0;
  long logMillis = 0;
  int logEncoder = 0;
  int logBattery = 0;

  while (1)
  {

    if (millis() - lastSent >= 20)
    {

      if (pulsantePremuto != !digitalRead(PULSANTE))
      { // cambio stato pulsante
        pulsantePremuto = !digitalRead(PULSANTE);
        if (pulsantePremuto)
        {
          buttonPressStart = millis();
        }
        else
        {
          if (millis() - buttonPressStart < BUTTON_LONGPRESS_TIME)
          {
            if (currentSystemState != STATE_CONFIGURAZIONE)
            {
              buttonPressed = true;
            }
          }
          else
          {
            buttonLongPressed = true;
          }

          buttonPressStart = 0;
        }
      }

      // capture variable state to log
      logState = currentSystemState;
      logPulses = currentPulses;
      logPosizione = getPercentagePosition() * 100;
      logPwm = power;
      logCurrent = currentFilter(adcCurrentToMilliAmp(analogRead(I_MOT)));
      logTarget = radSecToRpm(targetSpeed);
      logSpeed = radSecToRpm(currentSpeed);
      logMillis = millis();
      logEncoder = analogRead(POTENZIOMETRO);
      logBattery = voltageFilter(adcVoltageToBatteryVoltage(analogRead(ADC_BATT)));

      lastSent = millis();

      logSerial.print("{\"state\":");
      logSerial.print(logState);
      logSerial.print(",\"pulses\":");
      logSerial.print(logPulses);
      logSerial.print(",\"posizione\":");
      logSerial.print(logPosizione);
      logSerial.print(",\"pwm\":");
      logSerial.print(logPwm);
      logSerial.print(",\"current\":");
      logSerial.print(logCurrent);
      logSerial.print(",\"target\":");
      logSerial.print(logTarget);
      logSerial.print(",\"speed\":");
      logSerial.print(logSpeed);
      logSerial.print(",\"millis\":");
      logSerial.print(logMillis);
      logSerial.print(",\"encoder\":");
      logSerial.print(logEncoder);
      logSerial.print(",\"battery\":");
      logSerial.print(logBattery);
      logSerial.print("}\n");
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

      if (command.indexOf("stop") >= 0)
      {
        stopReceived = true;
        logSerial.println("stopOk");
      }

      if (command.indexOf("config0") >= 0)
      {
        startConfig = true;
        logSerial.println("startConfigOk");
      }

      if (command.indexOf("config1") >= 0)
      {
        endConfig = true;
        logSerial.println("endConfigOk");
      }

      if(command.indexOf("contTest0") >= 0)
      {
        continousTestActive = true;
        logSerial.println("Test continuo attivato");
      } 

      if(command.indexOf("contTest1") >= 0)
      {
        continousTestActive = false;
        logSerial.println("Test continuo disattivato");
      }

      if(command.indexOf("Get;") >= 0)
      {
        logSerial.printf("Get;%d;%d;%d;%d;%d;%d;%f;%f;%f;%f;%f;%f;%f;%f;\n",timeoutDuration, timeoutOpen, pulseStart, pulseEnd, rpmOpen, rpmClose, railStart, railEnd, kpOpen, kiOpen, kdOpen, kpClose, kiClose, kdClose);
        continue;
      }

      logSerial.println("Command received: " + command);

      // check if string contains Set
      if (command.indexOf("Set") < 0)
      {
        continue;
      }

      if (currentSystemState != STATE_INACTIVE)
      {
        logSerial.println("E' possibile modificare parametri solo da stato INACTIVE");
        continue;
      }

      // costanti controllo
      long tmptimeoutDuration = UNDEFINED_VALUE;
      long tmptimeoutOpen = UNDEFINED_VALUE;
      int tmppulseStart = UNDEFINED_VALUE;
      int tmppulseEnd = UNDEFINED_VALUE;
      int tmprpmOpen = UNDEFINED_VALUE;
      int tmprpmClose = UNDEFINED_VALUE;
      float tmprailStart = UNDEFINED_VALUE;
      float tmprailEnd = UNDEFINED_VALUE;

      // pid apertura
      float tmppidOpenKp = UNDEFINED_VALUE;
      float tmppidOpenKi = UNDEFINED_VALUE;
      float tmppidOpenKd = UNDEFINED_VALUE;

      // pid chiusura
      float tmppidCloseKp = UNDEFINED_VALUE;
      float tmppidCloseKi = UNDEFINED_VALUE;
      float tmppidCloseKd = UNDEFINED_VALUE;

      sscanf(command.c_str(), "Set;%d;%d;%d;%d;%d;%d;%f;%f;%f;%f;%f;%f;%f;%f;", &tmptimeoutDuration, &tmptimeoutOpen, &tmppulseStart, &tmppulseEnd, &tmprpmOpen, &tmprpmClose, &tmprailStart, &tmprailEnd, &tmppidOpenKp, &tmppidOpenKi, &tmppidOpenKd, &tmppidCloseKp, &tmppidCloseKi, &tmppidCloseKd);

      // TODO check input

      if (tmptimeoutDuration != UNDEFINED_VALUE)
      {
        timeoutDuration = tmptimeoutDuration;
      }
      if (tmptimeoutOpen != UNDEFINED_VALUE)
      {
        timeoutOpen = tmptimeoutOpen;
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

      // pid apertura
      if (tmppidOpenKp != UNDEFINED_VALUE)
      {
        kpOpen = tmppidOpenKp;
      }
      if (tmppidOpenKi != UNDEFINED_VALUE)
      {
        kiOpen = tmppidOpenKi;
      }
      if (tmppidOpenKd != UNDEFINED_VALUE)
      {
        kdOpen = tmppidOpenKd;
      }

      // pid chiusura
      if (tmppidCloseKp != UNDEFINED_VALUE)
      {
        kpClose = tmppidCloseKp;
      }
      if (tmppidCloseKi != UNDEFINED_VALUE)
      {
        kiClose = tmppidCloseKi;
      }
      if (tmppidCloseKd != UNDEFINED_VALUE)
      {
        kdClose = tmppidCloseKd;
      }

      logSerial.println("Parametri Modificati");
    }

    // vTaskDelay(1);
  }
}
