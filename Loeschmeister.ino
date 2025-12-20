/**
 * PROJEKT: LOESCHMEISTER ESP32 - FINAL STABLE 20.12.25
 */

#include <Adafruit_NeoPixel.h> 
#include <ESP32Servo.h>        
#include <WiFi.h>              
#include <WebServer.h>         
#include <Preferences.h>       

// ===============================================================================
// 1. HARDWARE-PIN-DEFINITIONEN 
// ===============================================================================
#define LED_PIN 18          
#define SERVO_LIFT_PIN 17   
#define SERVO_ROTATE_PIN 16 
#define POTI_PIN 34         
#define AKKU_PIN 35         
const int SENSOR_PINS[6] = {32, 33, 25, 26, 27, 14};
const byte PinENA = 23; 
const byte PinIN1 = 22; 
const byte PinIN2 = 21; 

// ===============================================================================
// 2. KONSTANTEN & EINSTELLUNGEN 
// ===============================================================================
const int NUM_GLAS_POSITIONS = 6;
const int NUM_TOTAL_POSITIONS = 7; 
const int REST_POSITION_INDEX = 6; 
const int NUM_PIXELS = 6;
const unsigned long FADE_UP_DURATION = 1000;  
const unsigned long DELAY_START_PROCESS = 2000; 
const unsigned long FADE_OUT_DURATION = 500;  
const unsigned long FINISH_TIME = 1000;      
const int BLAULICHT_PATTERN[] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, -1};
const long PULSE_DURATION = 55; 
const int PATTERN_STEPS = 17;
const int BASE_BLUE_BRIGHTNESS_MAX = 80;
const int BASE_BLUE_BRIGHTNESS_MIN = 1;
const int PULSE_SPEED_MS = 2000;

const float VOLTAGE_DIVIDER_RATIO = 6.7;  // Neuberechnunng des Wertes --> VOLTAGE_DIVIDER_RATIO(NEU) = VOLTAGE_DIVIDER_RATIO*(GEMESSENE_AKKUSPANNUNG/ANGEZEIGTER_WERT) --> Zur glättung des Wertes sollte ein 100uF Kondensator zwischen Masse ind dem Pin geschaltet werden.
const float ADC_REFERENCE_VOLTAGE = 3.3; 
const int ADC_MAX_VALUE = 4095; 
const float FULL_VOLTAGE = 16.8;        
const float WARNING_VOLTAGE = 14.0;     
const float LOW_VOLTAGE = 13.2;         
const unsigned long BATTERY_CHECK_INTERVAL = 5000; 

// ===============================================================================
// 3. GLOBALE VARIABLEN & STATUS
// ===============================================================================
Preferences preferences; 
WebServer server(80);    
const char* ap_ssid = "Loeschmeister_Konfig"; 
const char* ap_password = "Passwort123"; 

unsigned long lastBatteryCheckTime = 0; 
float currentBatteryVoltage = 0.0;     
bool isBatteryLow = false;              

int liftDownMicroSec = 500;
int liftUpMicroSec = 1500;
int rotationMicroSecs[NUM_TOTAL_POSITIONS] = {500, 800, 1100, 1400, 1700, 2000, 2300};
int microSecStep = 5;    
int stepDelayMs = 15;    
int restDelayMs = 10000; 
long minFillingTime = 500;
long maxFillingTime = 5000;
int pumpSpeed = 200;

long actualLiftUS;   
long actualRotateUS; 

enum LED_STATE { LED_OFF, LED_ACCEPTED, LED_RED, LED_RED_FADE_OUT, LED_RED_MANUAL_FADE_OUT, LED_BLUE_FLASH, LED_GREEN, LED_GREEN_FADE_OUT, LED_SOFT_RUN, LED_CRITICAL_LOW_BATT };
enum FILLING_STATE { PROCESS_IDLE, PROCESS_CONFIRMED, PROCESS_LIFT_UP, PROCESS_ROTATE, PROCESS_LIFT_DOWN, PROCESS_PUMP_ON, PROCESS_PUMP_OFF, PROCESS_COMPLETE, PROCESS_RETURN_LIFT_UP, PROCESS_RETURN_ROTATE, PROCESS_RETURN_LIFT_DOWN };
enum REST_RETURN_STATE { REST_IDLE, REST_LIFT_UP, REST_ROTATE, REST_LIFT_DOWN };

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo servoLift;
Servo servoRotate;

LED_STATE ledState[NUM_GLAS_POSITIONS];
FILLING_STATE processState[NUM_GLAS_POSITIONS] = {PROCESS_IDLE};
unsigned long startTime[NUM_GLAS_POSITIONS] = {0};

int currentProcessingPosition = -1; 
bool isSystemBusy = false;          
unsigned long lastActivityTime = 0; 
unsigned long lastServoStepTime = 0; 
unsigned long lastPatternChange = 0; 
int currentPatternIndexA = 0;        
int currentPatternIndexB = 8;        
int currentMechanismTargetAngle = -1; 
REST_RETURN_STATE restState = REST_IDLE; 

unsigned long globalPulseStartTime = 0; 
int testTargetPosition = -1; 

// ===============================================================================
// 4. SPEICHER & WEBSEITE
// ===============================================================================
void loadConfiguration() {
  preferences.begin("lox-config", true);
  liftDownMicroSec = preferences.getUInt("liftDownUS", liftDownMicroSec);
  liftUpMicroSec = preferences.getUInt("liftUpUS", liftUpMicroSec);
  stepDelayMs = preferences.getUInt("stepDelay", stepDelayMs);
  microSecStep = preferences.getUInt("microStep", microSecStep);
  restDelayMs = preferences.getUInt("restDelay", restDelayMs);
  pumpSpeed = preferences.getUInt("pumpSpeed", pumpSpeed);
  minFillingTime = preferences.getUInt("minFill", minFillingTime);
  maxFillingTime = preferences.getUInt("maxFill", maxFillingTime);
  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    char key[10];
    sprintf(key, "rot%d", i);
    rotationMicroSecs[i] = preferences.getUInt(key, rotationMicroSecs[i]);
  }
  preferences.end();
}

void saveConfiguration() {
  preferences.begin("lox-config", false);
  preferences.putUInt("liftDownUS", liftDownMicroSec);
  preferences.putUInt("liftUpUS", liftUpMicroSec);
  preferences.putUInt("stepDelay", stepDelayMs);
  preferences.putUInt("microStep", microSecStep);
  preferences.putUInt("restDelay", restDelayMs);
  preferences.putUInt("pumpSpeed", pumpSpeed);
  preferences.putUInt("minFill", minFillingTime);
  preferences.putUInt("maxFill", maxFillingTime);
  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    char key[10];
    sprintf(key, "rot%d", i);
    preferences.putUInt(key, rotationMicroSecs[i]);
  }
  preferences.end();
}

String generateConfigPage() {
  float batteryPct = constrain((currentBatteryVoltage - LOW_VOLTAGE) / (FULL_VOLTAGE - LOW_VOLTAGE) * 100.0, 0, 100);
  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Löschmeister</title>";
  html += "<style>";
  html += "body{font-family:sans-serif;padding:20px;background:#f0f0f0;}";
  html += ".box{background:white;padding:15px;border-radius:8px;margin-bottom:15px;box-shadow:0 2px 5px rgba(0,0,0,0.1);}";
  html += "label{display:inline-block;width:160px;}";
  html += "input{width:80px;padding:5px;margin-bottom:5px;}";
  html += ".test-btn{padding:5px 10px;background:#3f51b5;color:white;text-decoration:none;border-radius:4px;font-size:0.8em;}";
  html += ".batt-container{width:100%; background:#ddd; border-radius:5px;}";
  html += ".batt-bar{height:15px; width:" + String(batteryPct) + "%; background:" + (isBatteryLow ? "#f44336" : "#4CAF50") + "; border-radius:5px;}";
  html += ".red-flag{background:#f44336; color:white; padding:15px; border-radius:8px; text-align:center; font-weight:bold; margin-bottom:15px; animation: blink 1s infinite;}";
  html += "@keyframes blink{50%{opacity:0.5;}}";
  html += "</style></head>";
  html += "<body>";
  html += "<h1>🚒 Löschmeister Konfig</h1>";
  if (isBatteryLow) html += "<div class='red-flag'>⚠️ AKKU KRITISCH: " + String(currentBatteryVoltage, 2) + "V ⚠️</div>";
  html += "<div class='box'><b>Akku:</b> " + String(currentBatteryVoltage, 2) + "V (" + String((int)batteryPct) + "%)";
  html += "<div class='batt-container'><div class='batt-bar'></div></div></div>";
  html += "<form action='/save' method='post'>";
  html += "<div class='box'><h3>📏 Servos</h3>";
  html += "<label>Ab (us):</label><input type='number' name='liftDown' value='" + String(liftDownMicroSec) + "'><br>";
  html += "<label>Auf (us):</label><input type='number' name='liftUp' value='" + String(liftUpMicroSec) + "'></div>";
  html += "<div class='box'><h3>🌀 Pumpe & Poti</h3>";
  html += "<label>Pumpe Speed (0-255):</label><input type='range' name='pumpSpeed' min='0' max='255' value='" + String(pumpSpeed) + "' oninput='this.nextElementSibling.value = this.value'><output style='margin-left:10px; font-weight:bold;'>" + String(pumpSpeed) + "</output> <a href='/test?pump=1' class='test-btn' style='background:#f44336; margin-left:20px;'>Pumpe Test (2s)</a><br>";
  html += "<label>Poti Min (ms):</label><input type='number' name='minFill' value='" + String(minFillingTime) + "'><br>";
  html += "<label>Poti Max (ms):</label><input type='number' name='maxFill' value='" + String(maxFillingTime) + "'></div>";
  html += "<div class='box'><h3>📍 Glas-Positionen</h3>";
  for(int i = 0; i < 6; i++) html += "G" + String(i+1) + ": <input type='number' name='rot" + String(i) + "' value='" + String(rotationMicroSecs[i]) + "'> <a href='/test?pos=" + String(i) + "' class='test-btn'>Test</a><br>";
  html += "Ruhe: <input type='number' name='rot6' value='" + String(rotationMicroSecs[6]) + "'>";
  html += "<a href='/test?pos=6' class='test-btn'>Test</a></div>";
  html += "<input type='submit' value='💾 Speichern' style='padding:10px;width:100%;background:#2e7d32;color:white;border:none;border-radius:5px;'>";
  html += "</form></body></html>";
  return html;
}

void handleSave() {
  if (server.hasArg("liftDown")) liftDownMicroSec = server.arg("liftDown").toInt();
  if (server.hasArg("liftUp")) liftUpMicroSec = server.arg("liftUp").toInt();
  if (server.hasArg("pumpSpeed")) pumpSpeed = server.arg("pumpSpeed").toInt();
  if (server.hasArg("minFill")) minFillingTime = server.arg("minFill").toInt();
  if (server.hasArg("maxFill")) maxFillingTime = server.arg("maxFill").toInt();
  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    char arg[10]; sprintf(arg, "rot%d", i);
    if (server.hasArg(arg)) rotationMicroSecs[i] = server.arg(arg).toInt();
  }
  saveConfiguration();
  server.sendHeader("Location", "/", true); server.send(302, "text/plain", "");
}

void handleTestMove() {
  if (server.hasArg("pump")) {
    digitalWrite(PinIN1, HIGH);
    digitalWrite(PinIN2, LOW);
    ledcWrite(PinENA, pumpSpeed);
    delay(2000);
    ledcWrite(PinENA, 0);
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
    return;
  }
  if (server.hasArg("pos")) {
    int p = server.arg("pos").toInt();
    if (p >= 0 && p < NUM_TOTAL_POSITIONS) {
      currentMechanismTargetAngle = rotationMicroSecs[p]; 
      testTargetPosition = p;
      isSystemBusy = true;
      if (!servoLift.attached()) servoLift.attach(SERVO_LIFT_PIN);
      if (!servoRotate.attached()) servoRotate.attach(SERVO_ROTATE_PIN);
      restState = REST_LIFT_UP; 
      server.sendHeader("Location", "/", true);
      server.send(302, "text/plain", "");
      return;
    }
  }
  server.send(400, "text/plain", "Error");
}

// ===============================================================================
// 5. KERN-LOGIK
// ===============================================================================

void handleBatteryCheck(unsigned long currentMillis) {
  if (currentMillis - lastBatteryCheckTime < BATTERY_CHECK_INTERVAL) return;
  lastBatteryCheckTime = currentMillis;
  float v_out = (analogRead(AKKU_PIN) / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;
  currentBatteryVoltage = v_out * VOLTAGE_DIVIDER_RATIO;
  isBatteryLow = (currentBatteryVoltage <= LOW_VOLTAGE);
}

bool moveServoGradually(Servo &servo, long &actualValue, long targetValue, unsigned long currentMillis) {
  if (actualValue == targetValue) return true;
  if (currentMillis - lastServoStepTime >= (unsigned long)stepDelayMs) {
    lastServoStepTime = currentMillis;
    if (actualValue < targetValue) actualValue += microSecStep;
    else actualValue -= microSecStep;
    if (abs(targetValue - actualValue) < microSecStep) actualValue = targetValue;
    servo.writeMicroseconds(actualValue);
  }
  return (actualValue == targetValue);
}

void handleSensorLogic(int pos, unsigned long currentMillis) {
  bool isPresent = (digitalRead(SENSOR_PINS[pos]) == LOW);
  if (!isPresent && (ledState[pos] == LED_ACCEPTED || ledState[pos] == LED_RED || ledState[pos] == LED_RED_FADE_OUT || ledState[pos] == LED_BLUE_FLASH || ledState[pos] == LED_GREEN)) {
    if (currentProcessingPosition == pos && processState[pos] == PROCESS_PUMP_ON){
      ledcWrite(PinENA, 0);
    }
    ledState[pos] = LED_RED_MANUAL_FADE_OUT; startTime[pos] = currentMillis; 
    if (currentProcessingPosition == pos) {
      isSystemBusy = true; currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
      processState[pos] = PROCESS_RETURN_LIFT_UP;
    } else {
      processState[pos] = PROCESS_IDLE;
    }
  }
  if (isPresent && (ledState[pos] == LED_SOFT_RUN || ledState[pos] == LED_OFF)) { 
    ledState[pos] = LED_ACCEPTED;
    startTime[pos] = currentMillis; 
  }
  if ((ledState[pos] == LED_GREEN_FADE_OUT || ledState[pos] == LED_RED_MANUAL_FADE_OUT) && (currentMillis - startTime[pos] >= FADE_OUT_DURATION)) {
    ledState[pos] = LED_SOFT_RUN; 
  }
  switch (ledState[pos]) {
    case LED_ACCEPTED:
      if (currentMillis - startTime[pos] >= FADE_UP_DURATION) {
        ledState[pos] = LED_RED;
        startTime[pos] = currentMillis;
      }
    break;
    
    case LED_RED:
      if (currentMillis - startTime[pos] >= DELAY_START_PROCESS) {
        ledState[pos] = LED_RED_FADE_OUT;
        startTime[pos] = currentMillis;
      }
    break;
    
    case LED_RED_FADE_OUT:
      if (currentMillis - startTime[pos] >= FADE_OUT_DURATION) {
        ledState[pos] = LED_BLUE_FLASH;
        processState[pos] = PROCESS_CONFIRMED;
      }
    break;

    default:
    break;
  }
}

void handleFillingProcess(int pos, unsigned long currentMillis, long fillingDuration) {
  if (pos != currentProcessingPosition) return;
  if (processState[pos] != PROCESS_PUMP_ON && processState[pos] != PROCESS_IDLE) {
    if (!servoLift.attached()) servoLift.attach(SERVO_LIFT_PIN);
    if (!servoRotate.attached()) servoRotate.attach(SERVO_ROTATE_PIN);
  }
  
  switch (processState[pos]) {
    case PROCESS_CONFIRMED:
      processState[pos] = PROCESS_LIFT_UP;
    break;
    
    case PROCESS_LIFT_UP:
      if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, currentMillis)) {
        processState[pos] = PROCESS_ROTATE;
      }
    break;
    
    case PROCESS_ROTATE:
      if (moveServoGradually(servoRotate, actualRotateUS, rotationMicroSecs[pos], currentMillis)) {
        processState[pos] = PROCESS_LIFT_DOWN;
      }
    break;
    
    case PROCESS_LIFT_DOWN:
      if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, currentMillis)) {
        processState[pos] = PROCESS_PUMP_ON;
        digitalWrite(PinIN1, HIGH);
        digitalWrite(PinIN2, LOW);
        ledcWrite(PinENA, pumpSpeed);
        startTime[pos] = currentMillis;
      }
    break;
    
    case PROCESS_PUMP_ON:
      if (servoLift.attached()) servoLift.detach();
      if (servoRotate.attached()) servoRotate.detach();
      if (currentMillis - startTime[pos] >= (unsigned long)fillingDuration) {
        ledcWrite(PinENA, 0);
        ledState[pos] = LED_GREEN;
        processState[pos] = PROCESS_PUMP_OFF; 
        startTime[pos] = currentMillis; 
      }
    break;

    case PROCESS_PUMP_OFF:
      if (currentMillis - startTime[pos] >= FINISH_TIME) {
        servoLift.attach(SERVO_LIFT_PIN);
        servoRotate.attach(SERVO_ROTATE_PIN);
        int next = -1;
        for (int i = 0; i < 6; i++) if (processState[i] == PROCESS_CONFIRMED) {
          next = i;
          break;
        }
        currentMechanismTargetAngle = (next != -1) ? rotationMicroSecs[next] : rotationMicroSecs[REST_POSITION_INDEX];
        if (next != -1) currentProcessingPosition = next;
        processState[pos] = PROCESS_RETURN_LIFT_UP;
      }
    break;
    
    case PROCESS_RETURN_LIFT_UP:
      if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, currentMillis)) processState[pos] = PROCESS_RETURN_ROTATE;
    break;
    
    case PROCESS_RETURN_ROTATE:
      if (moveServoGradually(servoRotate, actualRotateUS, currentMechanismTargetAngle, currentMillis)) {
        if (currentMechanismTargetAngle == rotationMicroSecs[REST_POSITION_INDEX]) {
          processState[pos] = PROCESS_RETURN_LIFT_DOWN;
        } else {
          processState[pos] = PROCESS_COMPLETE;
          processState[currentProcessingPosition] = PROCESS_LIFT_DOWN;
        }
      }
    break;
    
    case PROCESS_RETURN_LIFT_DOWN:
      if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, currentMillis)) {
        processState[pos] = PROCESS_COMPLETE;
        isSystemBusy = false;
        currentProcessingPosition = -1;
        servoLift.detach();
        servoRotate.detach();
      }
    break;
    
    default:
    break;
  }
}

void handleRestingTimeout(unsigned long currentMillis) {
  if (currentProcessingPosition != -1) return;
  if (testTargetPosition == -1 && restState == REST_IDLE && !isSystemBusy) {
    if (actualLiftUS != liftDownMicroSec || actualRotateUS != rotationMicroSecs[REST_POSITION_INDEX]) {
      if (currentMillis - lastActivityTime >= (unsigned long)restDelayMs) {
        servoLift.attach(SERVO_LIFT_PIN);
        servoRotate.attach(SERVO_ROTATE_PIN);
        currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
        restState = REST_LIFT_UP;
      }
    }
  }
  if (restState != REST_IDLE) {
    switch (restState) {
      case REST_LIFT_UP:
        if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, currentMillis)) {
          restState = REST_ROTATE;
        }    
      break;
      
      case REST_ROTATE:
        if (moveServoGradually(servoRotate, actualRotateUS, currentMechanismTargetAngle, currentMillis)) {
          restState = REST_LIFT_DOWN;
        }
      break;
      
      case REST_LIFT_DOWN:
        if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, currentMillis)) { 
          restState = REST_IDLE;
          servoLift.detach();
          servoRotate.detach();
          if (testTargetPosition != -1) {
            isSystemBusy = false;
            testTargetPosition = -1;
          }
          lastActivityTime = currentMillis;
        }
      break;
    }
  }
}

void updateNeoPixels(unsigned long currentMillis) {
  int bluePulse = (int)(BASE_BLUE_BRIGHTNESS_MIN + ((-cos((float)(currentMillis - globalPulseStartTime) * 2.0 * PI / PULSE_SPEED_MS) + 1.0) / 2.0) * (BASE_BLUE_BRIGHTNESS_MAX - BASE_BLUE_BRIGHTNESS_MIN));
  for (int i = 0; i < 6; i++) {
    uint32_t color = 0;
    switch (ledState[i]) {
      case LED_CRITICAL_LOW_BATT:
        color = ((currentMillis / 250) % 2 == 0) ? strip.Color(255, 0, 0) : 0;
      break;

      case LED_SOFT_RUN:
        color = strip.Color(0, 0, bluePulse);
      break;
      
      case LED_ACCEPTED:
        color = strip.Color(map(currentMillis - startTime[i], 0, FADE_UP_DURATION, 0, 255), 0, map(currentMillis - startTime[i], 0, FADE_UP_DURATION, bluePulse, 0));
      break;

      case LED_RED:
        color = strip.Color(255, 0, 0);
      break;

      case LED_RED_FADE_OUT:
        color = strip.Color(map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 255, 0), 0, map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 0, bluePulse));
      break;

      case LED_RED_MANUAL_FADE_OUT:
        color = strip.Color(map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 255, 0), 0, 0);
      break;

      case LED_GREEN: color = strip.Color(0, map(currentMillis - startTime[i], 0, FINISH_TIME, 0, 255), 0); break;

      case LED_GREEN_FADE_OUT:
        color = strip.Color(0, map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 255, 0), 0);
      break;

      case LED_BLUE_FLASH:
        if (BLAULICHT_PATTERN[((i % 2 == 0) ? currentPatternIndexA : currentPatternIndexB)] == 1) color = strip.Color(0, 0, 255);
      break;

      default:
      break;
    }
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void setup() {
  Serial.begin(115200);
  loadConfiguration();
  
  WiFi.softAP(ap_ssid, ap_password);
  server.on("/", [](){ server.send(200, "text/html", generateConfigPage()); });
  server.on("/save", handleSave);
  server.on("/test", handleTestMove);
  server.begin();
  
  for (int i = 0; i < 6; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
    ledState[i] = LED_SOFT_RUN;
  }
  pinMode(POTI_PIN, INPUT);
  ledcAttach(PinENA, 5000, 8);
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  strip.begin();
  strip.show();
  ESP32PWM::allocateTimer(1);
  servoLift.attach(SERVO_LIFT_PIN);
  servoRotate.attach(SERVO_ROTATE_PIN);
  actualLiftUS = liftDownMicroSec;
  actualRotateUS = rotationMicroSecs[REST_POSITION_INDEX];
  currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
  restState = REST_LIFT_UP;
  globalPulseStartTime = millis();
  lastActivityTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  server.handleClient();
  handleBatteryCheck(currentMillis);
  if (currentMillis - lastPatternChange >= PULSE_DURATION) {
    lastPatternChange = currentMillis;
    currentPatternIndexA = (currentPatternIndexA + 1) % PATTERN_STEPS;
    currentPatternIndexB = (currentPatternIndexB + 1) % PATTERN_STEPS;
  }
  long dur = map(analogRead(POTI_PIN), 0, 4095, minFillingTime, maxFillingTime);
  if (isBatteryLow) {
    for (int i = 0; i < 6; i++) {
      ledState[i] = LED_CRITICAL_LOW_BATT;
    }
    ledcWrite(PinENA, 0);
  } else {
    for (int i = 0; i < 6; i++) {
      handleSensorLogic(i, currentMillis);
    }
    if (!isSystemBusy && currentProcessingPosition == -1) {
      for (int i = 0; i < 6; i++) if (processState[i] == PROCESS_CONFIRMED) {
        currentProcessingPosition = i;
        isSystemBusy = true; break;
      }
    }
  }
  handleFillingProcess(currentProcessingPosition, currentMillis, dur);
  handleRestingTimeout(currentMillis);
  updateNeoPixels(currentMillis);
}