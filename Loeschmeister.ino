/**
 * PROJEKT: LOESCHMEISTER ESP32 - FINAL STABLE 20.12.25
 * Beschreibung: Automatisierte Glasbefüllung mit Servo-Steuerung, 
 * Pumpenregelung, Akku-Überwachung und Web-Interface zur Konfiguration.
 */

#include <Adafruit_NeoPixel.h> // Steuerung der RGB-LEDs
#include <ESP32Servo.h>        // Servo-Bibliothek für ESP32
#include <WiFi.h>              // WLAN-Funktionalität
#include <WebServer.h>         // Lokaler Webserver für Einstellungen
#include <DNSServer.h>         // Captive Portal (automatische Umleitung zur Konfig)
#include <Preferences.h>       // Dauerhaftes Speichern von Werten im Flash-Speicher

// ===============================================================================
// 1. HARDWARE-PIN-DEFINITIONEN 
// ===============================================================================
#define LED_PIN 18          // Datenleitung für WS2812B NeoPixel
#define SERVO_LIFT_PIN 17   // Servo für die vertikale Bewegung (Heben/Senken)
#define SERVO_ROTATE_PIN 16 // Servo für die horizontale Drehung
#define POTI_PIN 34         // Analog-Eingang für Füllmengen-Potentiometer
#define AKKU_PIN 35         // Analog-Eingang für Batteriespannung (über Spannungsteiler)
const int SENSOR_PINS[6] = {32, 33, 25, 26, 27, 14}; // IR-Sensoren für Glas-Erkennung
const byte PinENA = 23;     // PWM-Pin für Motortreiber (Pumpengeschwindigkeit)
const byte PinIN1 = 22;     // Richtungspin 1 für Pumpe
const byte PinIN2 = 21;     // Richtungspin 2 für Pumpe

// ===============================================================================
// 2. KONSTANTEN & EINSTELLUNGEN 
// ===============================================================================
const int NUM_GLAS_POSITIONS = 6;
const int NUM_TOTAL_POSITIONS = 7; // 6 Gläser + 1 Ruheposition
const int REST_POSITION_INDEX = 6; // Index der Ruheposition im Array
const int NUM_PIXELS = 6;          // Anzahl der LEDs im Streifen

// Zeitkonstanten für Animationen und Prozesse (in ms)
const unsigned long FADE_UP_DURATION = 1000;  
const unsigned long DELAY_START_PROCESS = 2000; 
const unsigned long FADE_OUT_DURATION = 500;  
const unsigned long FINISH_TIME = 1000;      

// Blaulicht-Effekt (Feuerwehr-Simulation)
const int BLAULICHT_PATTERN[] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, -1};
const long PULSE_DURATION = 25; // Geschwindigkeit des Blitzens
const int PATTERN_STEPS = 17;
const int BASE_BLUE_BRIGHTNESS_MAX = 80;
const int BASE_BLUE_BRIGHTNESS_MIN = 1;
const int PULSE_SPEED_MS = 2000; // Dauer des sanften Pulsierens im Leerlauf

// Akku-Parameter für 4S LiPo (16.8V voll)
float VOLTAGE_DIVIDER_RATIO = 6.7;  // Neuberechnunng des Wertes --> VOLTAGE_DIVIDER_RATIO(NEU) = VOLTAGE_DIVIDER_RATIO*(GEMESSENE_AKKUSPANNUNG/ANGEZEIGTER_WERT) --> Zur glättung des Wertes sollte ein 100uF Kondensator zwischen Masse ind dem Pin geschaltet werden.
const float ADC_REFERENCE_VOLTAGE = 3.3; 
const int ADC_MAX_VALUE = 4095; 
const float FULL_VOLTAGE = 16.8;         
const float WARNING_VOLTAGE = 14.0;      
const float LOW_VOLTAGE = 13.2;         
const unsigned long BATTERY_CHECK_INTERVAL = 5000; 

// ===============================================================================
// 3. GLOBALE VARIABLEN & STATUS
// ===============================================================================
const char* ap_ssid = "Loeschmeister_Konfig"; 
const char* ap_password = "Passwort123"; 
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
IPAddress apSUBNET(255, 255, 255, 0);
DNSServer dnsServer;
Preferences preferences; 
WebServer server(80);    

unsigned long lastBatteryCheckTime = 0; 
float currentBatteryVoltage = 0.0;     
bool isBatteryLow = false;              

// Standardwerte für Servos (werden aus Preferences überschrieben)
int liftDownMicroSec = 500;
int liftUpMicroSec = 1500;
int rotationMicroSecs[NUM_TOTAL_POSITIONS] = {500, 800, 1100, 1400, 1700, 2000, 2300};
int microSecStep = 5;    // Schrittweite der Servobewegung (für sanften Lauf)
int stepDelayMs = 15;    // Verzögerung zwischen Schritten
int restDelayMs = 10000; // Zeit bis zur automatischen Rückkehr in Ruheposition
long minFillingTime = 500;
long maxFillingTime = 5000;
int pumpSpeed = 200;

long actualLiftUS;   // Aktuelle Position Lift-Servo in Mikrosekunden
long actualRotateUS; // Aktuelle Position Dreh-Servo in Mikrosekunden

// Zustands-Enumerationen für LED-Effekte und den mechanischen Ablauf
enum LED_STATE { LED_OFF, LED_ACCEPTED, LED_RED, LED_RED_FADE_OUT, LED_RED_MANUAL_FADE_OUT, LED_BLUE_FLASH, LED_GREEN, LED_GREEN_FADE_OUT, LED_SOFT_RUN, LED_CRITICAL_LOW_BATT };
enum FILLING_STATE { PROCESS_IDLE, PROCESS_CONFIRMED, PROCESS_LIFT_UP, PROCESS_ROTATE, PROCESS_LIFT_DOWN, PROCESS_PUMP_ON, PROCESS_PUMP_OFF, PROCESS_COMPLETE, PROCESS_RETURN_LIFT_UP, PROCESS_RETURN_ROTATE, PROCESS_RETURN_LIFT_DOWN };
enum REST_RETURN_STATE { REST_IDLE, REST_LIFT_UP, REST_ROTATE, REST_LIFT_DOWN };

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo servoLift;
Servo servoRotate;

LED_STATE ledState[NUM_GLAS_POSITIONS];
FILLING_STATE processState[NUM_GLAS_POSITIONS] = {PROCESS_IDLE};
unsigned long startTime[NUM_GLAS_POSITIONS] = {0};

int currentProcessingPosition = -1; // Welches Glas wird gerade befüllt?
bool isSystemBusy = false;          // Sperrt andere Prozesse während Bewegung
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

/**
 * Lädt alle gespeicherten Konfigurationswerte aus dem NVS-Speicher des ESP32.
 */
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
  VOLTAGE_DIVIDER_RATIO = preferences.getFloat("voltRatio", VOLTAGE_DIVIDER_RATIO);
  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    char key[10];
    sprintf(key, "rot%d", i);
    rotationMicroSecs[i] = preferences.getUInt(key, rotationMicroSecs[i]);
  }
  preferences.end();
}

/**
 * Speichert die aktuellen Werte (z.B. nach Änderung via Web-UI) im Flash.
 */
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
  preferences.putFloat("voltRatio", VOLTAGE_DIVIDER_RATIO);
  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    char key[10];
    sprintf(key, "rot%d", i);
    preferences.putUInt(key, rotationMicroSecs[i]);
  }
  preferences.end();
}

/**
 * Erzeugt das HTML für das Konfigurations-Interface.
 */
String generateConfigPage() {
  float batteryPct = constrain((currentBatteryVoltage - LOW_VOLTAGE) / (FULL_VOLTAGE - LOW_VOLTAGE) * 100.0, 0, 100);
  String html = "<!DOCTYPE html>";
  html += "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Löschmeister</title>";
  html += "<style>body{font-family:sans-serif;padding:20px;background:#f0f0f0;}.box{background:white;padding:15px;border-radius:8px;margin-bottom:15px;box-shadow:0 2px 5px rgba(0,0,0,0.1);}label{display:inline-block;width:160px;}input{width:80px;padding:5px;margin-bottom:5px;}.test-btn{padding:5px 10px;background:#3f51b5;color:white;text-decoration:none;border-radius:4px;font-size:0.8em;}.batt-container{width:100%; background:#ddd; border-radius:5px;}.batt-bar{height:15px; width:" + String(batteryPct) + "%; background:" + (isBatteryLow ? "#f44336" : "#4CAF50") + "; border-radius:5px;}.red-flag{background:#f44336; color:white; padding:15px; border-radius:8px; text-align:center; font-weight:bold; margin-bottom:15px; animation: blink 1s infinite;}@keyframes blink{50%{opacity:0.5;}}</style></head>";
  html += "<body><h1>🚒 Löschmeister Konfig</h1>";
  if (isBatteryLow) html += "<div class='red-flag'>⚠️ AKKU KRITISCH: " + String(currentBatteryVoltage, 2) + "V ⚠️</div>";
  html += "<div class='box'><b>Akku:</b> " + String(currentBatteryVoltage, 2) + "V (" + String((int)batteryPct) + "%)";
  html += "<div class='batt-container'><div class='batt-bar'></div></div><br>";
  // Neues Eingabefeld für den Voltage Divider Ratio
  html += "<form action='/save' method='post'><label>Volt Ratio:</label><input type='number' step='0.01' name='voltRatio' value='" + String(VOLTAGE_DIVIDER_RATIO) + "'></div>";
  
  html += "<div class='box'><h3>📏 Servos</h3>";
  html += "<label>Ab (us):</label><input type='number' name='liftDown' value='" + String(liftDownMicroSec) + "'><br>";
  html += "<label>Auf (us):</label><input type='number' name='liftUp' value='" + String(liftUpMicroSec) + "'><br>";
  html += "<label>Step Delay (ms):</label><input type='number' name='stepDelay' value='" + String(stepDelayMs) + "'><br>";
  html += "<label>Servo-Schrittweite:</label><input type='number' name='microStep' value='" + String(microSecStep) + "'></div>";

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

/**
 * Verarbeitet die vom Browser gesendeten Daten und speichert sie.
 */
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
  if (server.hasArg("stepDelay")) stepDelayMs = server.arg("stepDelay").toInt();
  if (server.hasArg("voltRatio")) VOLTAGE_DIVIDER_RATIO = server.arg("voltRatio").toFloat();
  if (server.hasArg("microStep")) microSecStep = server.arg("microStep").toInt();
  saveConfiguration();
  server.sendHeader("Location", "/", true); server.send(302, "text/plain", "");
}

/**
 * Ermöglicht das Testen einzelner Positionen oder der Pumpe über die Website.
 */
void handleTestMove() {
  if (server.hasArg("pump")) {
    digitalWrite(PinIN1, HIGH); digitalWrite(PinIN2, LOW);
    ledcWrite(PinENA, pumpSpeed); delay(2000); ledcWrite(PinENA, 0);
    server.sendHeader("Location", "/", true); server.send(302, "text/plain", "");
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
      server.sendHeader("Location", "/", true); server.send(302, "text/plain", "");
      return;
    }
  }
  server.send(400, "text/plain", "Error");
}

// ===============================================================================
// 5. KERN-LOGIK
// ===============================================================================

/**
 * Liest die Akkuspannung (Mittelwert aus 10 Messungen) und setzt den Status.
 */
void handleBatteryCheck(unsigned long currentMillis) {
  if (currentMillis - lastBatteryCheckTime < BATTERY_CHECK_INTERVAL) return;
  lastBatteryCheckTime = currentMillis;

  long adcSum = 0;
  
  // 10 Messungen durchführen
  for (int i = 0; i < 10; i++) {
    adcSum += analogRead(AKKU_PIN);
    delay(2); // Kurze Pause für die Stabilität des ADC
  }
  
  // Durchschnitt berechnen
  float averageADC = (float)adcSum / 10.0;
  
  // Umrechnung in Spannung
  float v_out = (averageADC / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;
  currentBatteryVoltage = v_out * VOLTAGE_DIVIDER_RATIO;
  
  isBatteryLow = (currentBatteryVoltage <= LOW_VOLTAGE);

  // Debug-Ausgabe für die Konsole (optional)
  Serial.print("Batteriespannung: ");
  Serial.print(currentBatteryVoltage);
  Serial.println(" V");
}
/**
 * Bewegt einen Servo ruckelfrei in kleinen Schritten zum Zielwert.
 * @return true, wenn das Ziel erreicht wurde.
 */
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

/**
 * Überwacht die IR-Sensoren und steuert die Zustandsübergänge für jedes Glas.
 */
void handleSensorLogic(int pos, unsigned long currentMillis) {
  bool isPresent = (digitalRead(SENSOR_PINS[pos]) == LOW); // LOW bedeutet Glas erkannt
  
  // Wenn Glas entfernt wurde während ein Prozess lief
  if (!isPresent && (ledState[pos] == LED_ACCEPTED || ledState[pos] == LED_RED || ledState[pos] == LED_RED_FADE_OUT || ledState[pos] == LED_BLUE_FLASH )) {
    if (currentProcessingPosition == pos && processState[pos] == PROCESS_PUMP_ON){
      ledcWrite(PinENA, 0); // Not-Aus Pumpe
    }
    ledState[pos] = LED_RED_MANUAL_FADE_OUT;
    startTime[pos] = currentMillis; 

    if (currentProcessingPosition == pos) {
      isSystemBusy = true;
      currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
      processState[pos] = PROCESS_RETURN_LIFT_UP; // Mechanik zurückfahren
    } else {
      processState[pos] = PROCESS_IDLE;
    }
  } else if (!isPresent && ledState[pos] == LED_GREEN) { // Glas ist voll und wird entnommen
    ledState[pos] = LED_GREEN_FADE_OUT; 
    startTime[pos] = currentMillis; // Timer für den Fade-Vorgang starten
  }

  // Neues Glas erkannt
  if (isPresent && (ledState[pos] == LED_SOFT_RUN || ledState[pos] == LED_OFF)) { 
    ledState[pos] = LED_ACCEPTED;
    startTime[pos] = currentMillis; 
  }

  // Timer-basierte Übergänge der LED-Animationen (Blau -> Rot -> Blinken -> Füllen)
  if ((ledState[pos] == LED_RED_MANUAL_FADE_OUT || ledState[pos] == LED_GREEN_FADE_OUT) && (currentMillis - startTime[pos] >= FADE_OUT_DURATION)) {
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
        processState[pos] = PROCESS_CONFIRMED; // Glas ist bereit zum Befüllen
      }
    break;
    default: break;
  }
}

/**
 * Die zentrale Zustandsmaschine für den mechanischen Füllvorgang.
 */
void handleFillingProcess(int pos, unsigned long currentMillis, long fillingDuration) {
  if (pos != currentProcessingPosition) return;

  // Servos nur ansteuern, wenn wir nicht gerade nur pumpen (Strom sparen/Zittern vermeiden)
  if (processState[pos] != PROCESS_PUMP_ON && processState[pos] != PROCESS_IDLE) {
    if (!servoLift.attached()) servoLift.attach(SERVO_LIFT_PIN);
    if (!servoRotate.attached()) servoRotate.attach(SERVO_ROTATE_PIN);
  }
  
  switch (processState[pos]) {
    case PROCESS_CONFIRMED:
      processState[pos] = PROCESS_LIFT_UP;
    break;
    case PROCESS_LIFT_UP: // Erst Arm heben
      if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, currentMillis)) processState[pos] = PROCESS_ROTATE;
    break;
    case PROCESS_ROTATE: // Dann zum Glas drehen
      if (moveServoGradually(servoRotate, actualRotateUS, rotationMicroSecs[pos], currentMillis)) processState[pos] = PROCESS_LIFT_DOWN;
    break;
    case PROCESS_LIFT_DOWN: // Arm ins Glas senken
      if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, currentMillis)) {
        processState[pos] = PROCESS_PUMP_ON;
        digitalWrite(PinIN1, HIGH); digitalWrite(PinIN2, LOW);
        ledcWrite(PinENA, pumpSpeed); // Pumpe Start
        startTime[pos] = currentMillis;
      }
    break;
    case PROCESS_PUMP_ON: // Befüllung läuft
      //if (servoLift.attached()) servoLift.detach(); // Servos lösen zur Rauschunterdrückung
      //if (servoRotate.attached()) servoRotate.detach();
      if (currentMillis - startTime[pos] >= (unsigned long)fillingDuration) {
        ledcWrite(PinENA, 0); // Pumpe Stopp
        ledState[pos] = LED_GREEN;
        processState[pos] = PROCESS_PUMP_OFF; 
        startTime[pos] = currentMillis; 
      }
    break;
    case PROCESS_PUMP_OFF: // Kurze Wartezeit nach dem Pumpen (Nachtropfen)
      if (currentMillis - startTime[pos] >= FINISH_TIME) {
        servoLift.attach(SERVO_LIFT_PIN);
        servoRotate.attach(SERVO_ROTATE_PIN);
        // Prüfen ob noch ein Glas wartet
        int next = -1;
        for (int i = 0; i < 6; i++) if (processState[i] == PROCESS_CONFIRMED) { next = i; break; }
        currentMechanismTargetAngle = (next != -1) ? rotationMicroSecs[next] : rotationMicroSecs[REST_POSITION_INDEX];
        if (next != -1) currentProcessingPosition = next;
        processState[pos] = PROCESS_RETURN_LIFT_UP;
      }
    break;
    case PROCESS_RETURN_LIFT_UP:
      if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, currentMillis)) processState[pos] = PROCESS_RETURN_ROTATE;
    break;
    case PROCESS_RETURN_ROTATE: // Zur nächsten Position oder Ruheposition drehen
      if (moveServoGradually(servoRotate, actualRotateUS, currentMechanismTargetAngle, currentMillis)) {
        if (currentMechanismTargetAngle == rotationMicroSecs[REST_POSITION_INDEX]) {
          processState[pos] = PROCESS_RETURN_LIFT_DOWN;
        } else {
          processState[pos] = PROCESS_COMPLETE; // Direkt weiter zum nächsten Glas
          processState[currentProcessingPosition] = PROCESS_LIFT_DOWN;
        }
      }
    break;
    case PROCESS_RETURN_LIFT_DOWN:
      if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, currentMillis)) {
        processState[pos] = PROCESS_COMPLETE;
        isSystemBusy = false; currentProcessingPosition = -1;
        delay(200);
        servoLift.detach(); servoRotate.detach();
      }
    break;
    default: break;
  }
}

/**
 * Bewegt die Mechanik in die Parkposition, wenn eine Zeit lang nichts passiert ist.
 */
void handleRestingTimeout(unsigned long currentMillis) {
  if (currentProcessingPosition != -1) return;
  if (testTargetPosition == -1 && restState == REST_IDLE && !isSystemBusy) {
    if (actualLiftUS != liftDownMicroSec || actualRotateUS != rotationMicroSecs[REST_POSITION_INDEX]) {
      if (currentMillis - lastActivityTime >= (unsigned long)restDelayMs) {
        servoLift.attach(SERVO_LIFT_PIN); servoRotate.attach(SERVO_ROTATE_PIN);
        currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
        restState = REST_LIFT_UP;
      }
    }
  }
  // Ablaufsteuerung für die Ruhe-Fahrt
  if (restState != REST_IDLE) {
    switch (restState) {
      case REST_LIFT_UP:
        if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, currentMillis)) restState = REST_ROTATE;
      break;
      case REST_ROTATE:
        if (moveServoGradually(servoRotate, actualRotateUS, currentMechanismTargetAngle, currentMillis)) restState = REST_LIFT_DOWN;
      break;
      case REST_LIFT_DOWN:
        if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, currentMillis)) { 
          restState = REST_IDLE;
          delay(200);
          servoLift.detach(); servoRotate.detach();
          if (testTargetPosition != -1) { isSystemBusy = false; testTargetPosition = -1; }
          lastActivityTime = currentMillis;
        }
      break;
    }
  }
}

/**
 * Berechnet die Farben der NeoPixel basierend auf dem aktuellen Zustand (Fades, Blinken, Pulsieren).
 */
void updateNeoPixels(unsigned long currentMillis) {
  // Sinus-Berechnung für das sanfte blaue Pulsieren im Leerlauf
  int bluePulse = (int)(BASE_BLUE_BRIGHTNESS_MIN + ((-cos((float)(currentMillis - globalPulseStartTime) * 2.0 * PI / PULSE_SPEED_MS) + 1.0) / 2.0) * (BASE_BLUE_BRIGHTNESS_MAX - BASE_BLUE_BRIGHTNESS_MIN));
  
  for (int i = 0; i < 6; i++) {
    uint32_t color = 0;
    switch (ledState[i]) {
      case LED_CRITICAL_LOW_BATT: // Schnelles rotes Blinken bei leerem Akku
        color = ((currentMillis / 250) % 2 == 0) ? strip.Color(255, 0, 0) : 0;
      break;
      case LED_SOFT_RUN: // Normalzustand (Pulsieren)
        color = strip.Color(0, 0, bluePulse);
      break;
      case LED_ACCEPTED: // Übergang von Blau nach Rot (Glas wurde erkannt)
        color = strip.Color(map(currentMillis - startTime[i], 0, FADE_UP_DURATION, 0, 255), 0, map(currentMillis - startTime[i], 0, FADE_UP_DURATION, bluePulse, 0));
      break;
      case LED_RED: color = strip.Color(255, 0, 0); break;
      case LED_RED_FADE_OUT: // Übergang von Rot zurück nach Blau (kurz vor Start)
        color = strip.Color(map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 255, 0), 0, map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 0, bluePulse));
      break;
      case LED_RED_MANUAL_FADE_OUT: // Glas entnommen während Alarm
        color = strip.Color(map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 255, 0), 0, 0);
      break;
      case LED_GREEN: // Fertig befüllt
        color = strip.Color(0, bluePulse, 0); 
      break;
      case LED_GREEN_FADE_OUT:
        color = strip.Color(0, map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, bluePulse, 0), map(currentMillis - startTime[i], 0, FADE_OUT_DURATION, 0, bluePulse));
      break;
      case LED_BLUE_FLASH: // Blaulicht-Effekt während des Füllens
        if (BLAULICHT_PATTERN[((i % 2 == 0) ? currentPatternIndexA : currentPatternIndexB)] == 1) color = strip.Color(0, 0, 255);
      break;
      default: break;
    }
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// ===============================================================================
// 6. SETUP & LOOP
// ===============================================================================

void setup() {
  Serial.begin(115200);
  loadConfiguration();
  
  // WLAN Access Point und Captive Portal (DNS) einrichten
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, apSUBNET);
  WiFi.softAP(ap_ssid, ap_password, 1, 0, 1);
  dnsServer.start(DNS_PORT, "*", apIP);
  
  // Routen für Webserver definieren
  server.on("/", [](){server.send(200, "text/html", generateConfigPage()); });
  server.on("/save", handleSave);
  server.on("/test", handleTestMove);
  server.onNotFound([](){server.send(200, "text/html", generateConfigPage()); });
  server.begin();
  
  // IO-Pins initialisieren
  for (int i = 0; i < 6; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
    ledState[i] = LED_SOFT_RUN;
  }
  pinMode(POTI_PIN, INPUT);
  ledcAttach(PinENA, 5000, 8); // PWM für Pumpe (5kHz, 8-Bit)
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  
  strip.begin();
  strip.show();
  
  // Servo-Setup
  ESP32PWM::allocateTimer(1);
  servoLift.attach(SERVO_LIFT_PIN);
  servoRotate.attach(SERVO_ROTATE_PIN);
  
  // Start-Position anfahren
  actualLiftUS = liftDownMicroSec;
  actualRotateUS = rotationMicroSecs[REST_POSITION_INDEX];
  currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
  restState = REST_LIFT_UP;
  
  globalPulseStartTime = millis();
  lastActivityTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Hintergrund-Services
  dnsServer.processNextRequest(); 
  server.handleClient();    
  handleBatteryCheck(currentMillis);
  
  // Blaulicht-Taktgeber
  if (currentMillis - lastPatternChange >= PULSE_DURATION) {
    lastPatternChange = currentMillis;
    currentPatternIndexA = (currentPatternIndexA + 1) % PATTERN_STEPS;
    currentPatternIndexB = (currentPatternIndexB + 1) % PATTERN_STEPS;
  }
  
  // Fülldauer vom Poti einlesen
  long dur = map(analogRead(POTI_PIN), 0, 4095, minFillingTime, maxFillingTime);
  
  if (isBatteryLow) {
    // Bei schwachem Akku System sperren und warnen
    for (int i = 0; i < 6; i++) ledState[i] = LED_CRITICAL_LOW_BATT;
    ledcWrite(PinENA, 0);
  } else {
    // Sensoren prüfen
    for (int i = 0; i < 6; i++) handleSensorLogic(i, currentMillis);
    
    // Wenn nichts zu tun ist, nach neuem Auftrag (Glas) suchen
    if (!isSystemBusy && currentProcessingPosition == -1) {
      for (int i = 0; i < 6; i++) if (processState[i] == PROCESS_CONFIRMED) {
        currentProcessingPosition = i;
        isSystemBusy = true; break;
      }
    }
  }
  
  // Prozesse abarbeiten
  handleFillingProcess(currentProcessingPosition, currentMillis, dur);
  handleRestingTimeout(currentMillis);
  updateNeoPixels(currentMillis);
}