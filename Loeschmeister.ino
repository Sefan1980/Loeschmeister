/**
 * PROJEKT: LOESCHMEISTER ESP32
 * BESCHREIBUNG: Automatischer Getränke-Ausschenker mit Web-Konfiguration.
 * FUNKTION: Erkennt bis zu 6 Gläser, fährt diese sequenziell an und befüllt sie.
 * * AKTUELLER STATUS: 
 * - UI-Verbesserung: Glas-Positionen sind nun durch Rahmen (Fieldsets) visuell gruppiert.
 * - Speichern (Save) und Testen (Anfahren) leiten nun automatisch zur Konfigurationsseite zurück.
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
const int BASE_BLUE_BRIGHTNESS_MAX = 100;
const int BASE_BLUE_BRIGHTNESS_MIN = 5;
const int PULSE_SPEED_MS = 2000;

// ===============================================================================
// 3. GLOBALE VARIABLEN & STATUS-SPEICHER
// ===============================================================================
Preferences preferences; 
WebServer server(80);    
const char* ap_ssid = "Loeschmeister_Konfig"; 
const char* ap_password = "Passwort123"; 

// --- Konfigurationsvariablen (werden aus Flash geladen) ---
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

enum LED_STATE { LED_OFF, LED_ACCEPTED, LED_RED, LED_RED_FADE_OUT, LED_RED_MANUAL_FADE_OUT, LED_BLUE_FLASH, LED_GREEN, LED_GREEN_FADE_OUT, LED_SOFT_RUN };
enum FILLING_STATE { PROCESS_IDLE, PROCESS_CONFIRMED, PROCESS_LIFT_UP, PROCESS_ROTATE, PROCESS_LIFT_DOWN, PROCESS_PUMP_ON, PROCESS_PUMP_OFF, PROCESS_COMPLETE, PROCESS_RETURN_LIFT_UP, PROCESS_RETURN_ROTATE, PROCESS_RETURN_LIFT_DOWN };
enum REST_RETURN_STATE { REST_IDLE, REST_LIFT_UP, REST_ROTATE, REST_LIFT_DOWN };

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo servoLift;
Servo servoRotate;

LED_STATE ledState[NUM_GLAS_POSITIONS] = {LED_OFF};
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
// 4. SPEICHER- & WEBSERVER-FUNKTIONEN 
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
    char key[10]; sprintf(key, "rot%d", i);
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
    char key[10]; sprintf(key, "rot%d", i);
    preferences.putUInt(key, rotationMicroSecs[i]);
  }
  preferences.end();
}

String generateConfigPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Löschmeister Konfig</title>";
  // NEUE CSS für Fieldset/Legend
  html += "<style>body{font-family:sans-serif;padding:20px;background:#f0f0f0;} h1{color:#d32f2f;} .box{background:white;padding:15px;border-radius:8px;margin-bottom:15px;box-shadow:0 2px 5px rgba(0,0,0,0.1);} label{display:inline-block;width:180px;margin-bottom:8px;} input{width:80px;padding:5px;} .test-btn{padding:5px 10px;background:#3f51b5;color:white;text-decoration:none;border-radius:4px;font-size:0.8em;margin-left:10px;}";
  html += "input[type='submit'] { padding:10px 20px; background:#2e7d32; color:white; border:none; border-radius:5px; cursor:pointer; width:auto; min-width:150px; }";
  html += "fieldset{border:1px solid #ccc; border-radius:6px; padding:10px 15px; margin-bottom:15px;} legend{font-weight:bold; color:#3f51b5; padding:0 10px;}"; // NEU
  html += "</style></head><body>";
  html += "<h1>🚒 Löschmeister Konfiguration</h1><form action='/save' method='post'>";
  
  html += "<div class='box'><h2>📏 Servo-Einstellungen</h2>";
  html += "<label>Leiter AB (us):</label><input type='number' name='liftDown' value='" + String(liftDownMicroSec) + "'><br>";
  html += "<label>Leiter AUF (us):</label><input type='number' name='liftUp' value='" + String(liftUpMicroSec) + "'><br>";
  html += "<label>Schrittweite (us):</label><input type='number' name='microStep' value='" + String(microSecStep) + "'><br>";
  html += "<label>Verzögerung (ms):</label><input type='number' name='stepDelay' value='" + String(stepDelayMs) + "'></div>";

  html += "<div class='box'><h2>📍 Glas-Positionen</h2>";
  
  // Gruppierung für jede Glasposition (0-5)
  for(int i = 0; i < 6; i++) {
    html += "<fieldset><legend>Glas " + String(i+1) + " (Position " + String(i) + ")</legend>";
    html += "<label>Mikrosekunden (us):</label><input type='number' name='rot" + String(i) + "' value='" + String(rotationMicroSecs[i]) + "'>";
    html += "<a href='/test?pos=" + String(i) + "' class='test-btn' target='_blank'>Anfahren</a>";
    html += "</fieldset>";
  }
  
  // Gruppierung für die Ruheposition (6)
  html += "<fieldset><legend>Ruheposition (Position 6)</legend>";
  html += "<label>Mikrosekunden (us):</label><input type='number' name='rot6' value='" + String(rotationMicroSecs[6]) + "'>";
  html += "<a href='/test?pos=6' class='test-btn' target='_blank'>Anfahren</a>";
  html += "</fieldset></div>"; 

  html += "<div class='box'><h2>🌀 Pumpe</h2><label>Speed (0-255):</label><input type='number' name='pumpSpeed' value='" + String(pumpSpeed) + "'><br>";
  html += "<label>Ruherückkehr Wartezeit (ms):</label><input type='number' name='restDelay' value='" + String(restDelayMs) + "'></div>";
  
  html += "<input type='submit' value='💾 Alles Speichern'>";
  html += "</form></body></html>";
  return html;
}

void handleSave() {
  if (server.hasArg("liftDown")) liftDownMicroSec = server.arg("liftDown").toInt();
  if (server.hasArg("liftUp")) liftUpMicroSec = server.arg("liftUp").toInt();
  if (server.hasArg("stepDelay")) stepDelayMs = server.arg("stepDelay").toInt();
  if (server.hasArg("microStep")) microSecStep = server.arg("microStep").toInt();
  if (server.hasArg("pumpSpeed")) pumpSpeed = server.arg("pumpSpeed").toInt();
  if (server.hasArg("restDelay")) restDelayMs = server.arg("restDelay").toInt();
  
  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    char arg[10]; sprintf(arg, "rot%d", i);
    if (server.hasArg(arg)) rotationMicroSecs[i] = server.arg(arg).toInt();
  }
  
  saveConfiguration();
  
  String response = "Die Werte wurden gespeichert. Bitte starten Sie den ESP32 neu, um sie zu aktivieren.";
  
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", response);
}

void handleTestMove() {
  if (server.hasArg("pos")) {
    int p = server.arg("pos").toInt();
    if (p >= 0 && p < NUM_TOTAL_POSITIONS) {
      
      isSystemBusy = true; 
      currentProcessingPosition = -1; 
      testTargetPosition = p; 
      currentMechanismTargetAngle = rotationMicroSecs[p]; 
      
      if (!servoLift.attached()) servoLift.attach(SERVO_LIFT_PIN);
      if (!servoRotate.attached()) servoRotate.attach(SERVO_ROTATE_PIN);

      actualLiftUS = liftDownMicroSec; 
      restState = REST_LIFT_UP; 
      
      server.sendHeader("Location", "/", true);
      server.send(302, "text/plain", "Weiterleitung zur Konfigurationsseite.");
      
      return;
    }
  }
  server.send(400, "text/plain", "Fehler: Ungültige Position (0-6).");
}

// ===============================================================================
// 5. KERN-LOGIK: BEWEGUNG & SENSOREN
// ===============================================================================

// Funktion zur sanften, schrittweisen Servobewegung
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

// Überwacht die Glas-Sensoren und steuert die LED-Farben
void handleSensorLogic(int pos, unsigned long currentMillis) {
  bool isPresent = (digitalRead(SENSOR_PINS[pos]) == LOW);
  
  bool isWaitingForService = (ledState[pos] == LED_ACCEPTED || ledState[pos] == LED_RED || ledState[pos] == LED_RED_FADE_OUT || ledState[pos] == LED_BLUE_FLASH);
  
  bool removedDuringWait = !isPresent && isWaitingForService; 
  
  // Zustandswechsel: Grün -> Ausfaden (entfernt)
  if (!isPresent && ledState[pos] == LED_GREEN) {
      ledState[pos] = LED_GREEN_FADE_OUT; 
      startTime[pos] = currentMillis;
  }
  
  // Zustandswechsel: Fade Out Ende -> Blau Pulsieren
  if ((ledState[pos] == LED_GREEN_FADE_OUT || ledState[pos] == LED_RED_MANUAL_FADE_OUT) && currentMillis - startTime[pos] >= FADE_OUT_DURATION) {
      ledState[pos] = LED_SOFT_RUN; 
  }
  
  // Zustandswechsel: Blau Pulsieren -> Rot/Füllen (Glas erkannt)
  if ((ledState[pos] == LED_SOFT_RUN || ledState[pos] == LED_OFF) && isPresent) { 
      ledState[pos] = LED_ACCEPTED; 
      startTime[pos] = currentMillis; 
  }

  // --- GLAS ENTFERNT (Während es auf Bedienung wartete) ---
  if (removedDuringWait) {
    if (processState[pos] == PROCESS_PUMP_ON) ledcWrite(PinENA, 0);
    
    ledState[pos] = LED_RED_MANUAL_FADE_OUT; 
    startTime[pos] = currentMillis; 
    
    if (currentProcessingPosition == pos) {
      isSystemBusy = true;
      currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
      processState[pos] = PROCESS_RETURN_LIFT_UP;
    } else {
      processState[pos] = PROCESS_IDLE;
    }
    lastActivityTime = currentMillis;
    return;
  }
  // ---------------------------------------------------

  // Ablauf der Glas-Erkennung (Farben-Logik)
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
    default: break;
  }
}

// Steuert den Füllprozess
void handleFillingProcess(int pos, unsigned long currentMillis, long fillingDuration) {
  if (pos != currentProcessingPosition) return;

  switch (processState[pos]) {
    case PROCESS_CONFIRMED:
      if (!servoLift.attached()) servoLift.attach(SERVO_LIFT_PIN);
      if (!servoRotate.attached()) servoRotate.attach(SERVO_ROTATE_PIN);
      processState[pos] = PROCESS_LIFT_UP;
      break;

    case PROCESS_LIFT_UP:
      if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, currentMillis)) processState[pos] = PROCESS_ROTATE;
      break;

    case PROCESS_ROTATE:
      if (moveServoGradually(servoRotate, actualRotateUS, rotationMicroSecs[pos], currentMillis)) processState[pos] = PROCESS_LIFT_DOWN;
      break;

    case PROCESS_LIFT_DOWN:
      if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, currentMillis)) {
        processState[pos] = PROCESS_PUMP_ON;
        digitalWrite(PinIN1, HIGH); digitalWrite(PinIN2, LOW);
        ledcWrite(PinENA, pumpSpeed);
        startTime[pos] = currentMillis;
      }
      break;

    case PROCESS_PUMP_ON:
      if (currentMillis - startTime[pos] >= (unsigned long)fillingDuration) {
        ledcWrite(PinENA, 0);
        ledState[pos] = LED_GREEN; 
        processState[pos] = PROCESS_PUMP_OFF;
        startTime[pos] = currentMillis; 
      }
      break;

    case PROCESS_PUMP_OFF:
      if (currentMillis - startTime[pos] >= FINISH_TIME) {
        int next = -1;
        for (int i = 0; i < 6; i++) { if (processState[i] == PROCESS_CONFIRMED) { next = i; break; } }
        
        if (next != -1) {
          currentMechanismTargetAngle = rotationMicroSecs[next];
          currentProcessingPosition = next;
        } else {
          currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
        }
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

    default: break;
  }
}

// Handhabt nun sowohl den Ruhe-Timeout als auch die Testbewegungen
void handleRestingTimeout(unsigned long currentMillis) {
  // Wenn ein Füllprozess aktiv ist, nicht eingreifen
  if (currentProcessingPosition != -1) return;

  // 1. Logik: Automatischer Ruhe-Timeout
  // Nur aktiv, wenn KEIN Test läuft.
  if (testTargetPosition == -1) { 
    if (restState == REST_IDLE && !isSystemBusy) {
      // Prüfe, ob wir nicht bereits in der Ruheposition sind
      if (actualLiftUS != liftDownMicroSec || actualRotateUS != rotationMicroSecs[REST_POSITION_INDEX]) {
        if (currentMillis - lastActivityTime >= (unsigned long)restDelayMs) {
          if (!servoLift.attached()) servoLift.attach(SERVO_LIFT_PIN);
          if (!servoRotate.attached()) servoRotate.attach(SERVO_ROTATE_PIN);
          currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
          restState = REST_LIFT_UP;
        }
      }
    }
  }

  // 2. Abarbeitung der Servo-Bewegung (gilt für Setup, Timeout und Testmodus)
  if (restState != REST_IDLE) {
      
      unsigned long stepTime = currentMillis; 

      switch (restState) {
          case REST_LIFT_UP: 
              if (moveServoGradually(servoLift, actualLiftUS, liftUpMicroSec, stepTime)) restState = REST_ROTATE; 
              break;
          case REST_ROTATE: 
              if (moveServoGradually(servoRotate, actualRotateUS, currentMechanismTargetAngle, stepTime)) restState = REST_LIFT_DOWN; 
              break;
          case REST_LIFT_DOWN: 
              if (moveServoGradually(servoLift, actualLiftUS, liftDownMicroSec, stepTime)) { 
                  restState = REST_IDLE; 
                  servoLift.detach(); servoRotate.detach();
                  
                  // Reset der Testvariablen und des Busy-Zustands, wenn es ein Test war
                  if (testTargetPosition != -1) {
                      isSystemBusy = false;
                      testTargetPosition = -1;
                  }
                  lastActivityTime = currentMillis; // Wichtig: Setze die Aktivitätszeit zurück, um den nächsten Timeout zu starten
              } 
              break;
          default: break;
      }
  }
}

// Stellt sicher, dass LEDs im Ruhezustand auf Blau Pulsieren gesetzt werden
void handleRestingLED(unsigned long currentMillis) {
  if (globalPulseStartTime == 0) {
      globalPulseStartTime = currentMillis;
  }

  for(int i = 0; i < NUM_GLAS_POSITIONS; i++) {
    if (ledState[i] == LED_OFF) {
        ledState[i] = LED_SOFT_RUN;
    }
  }
}

// Berechnet den Blauen Puls-Faktor synchron für alle LEDs
int calculatePulsingBlue(unsigned long currentMillis) {
    unsigned long elapsed = currentMillis - globalPulseStartTime;
    
    float frequency = 2.0 * PI / (float)PULSE_SPEED_MS;
    float cosValue = cos((float)elapsed * frequency);
    float waveValue = -cosValue; 
    float pulseFactor = (waveValue + 1.0) / 2.0; 
    
    int span = BASE_BLUE_BRIGHTNESS_MAX - BASE_BLUE_BRIGHTNESS_MIN;
    
    return (int)(BASE_BLUE_BRIGHTNESS_MIN + pulseFactor * span);
}


// Zeichnet die LED-Farben basierend auf dem Status
void updateNeoPixels(unsigned long currentMillis) {
  
  for (int i = 0; i < 6; i++) {
    uint32_t color = strip.Color(0,0,0);
    
    switch (ledState[i]) {
        case LED_SOFT_RUN: { 
            int B = calculatePulsingBlue(currentMillis); 
            color = strip.Color(0, 0, B);
            break;
        }
        case LED_ACCEPTED: {
            unsigned long elapsed = currentMillis - startTime[i];
            
            int R_brightness;
            R_brightness = map(elapsed, 0, FADE_UP_DURATION, 0, 255);
            R_brightness = constrain(R_brightness, 0, 255);
            
            int B_fade;
            B_fade = map(elapsed, 0, FADE_UP_DURATION, BASE_BLUE_BRIGHTNESS_MAX, 0);
            B_fade = constrain(B_fade, 0, 255); 
            
            color = strip.Color(R_brightness, 0, B_fade);
            break;
        }
        case LED_RED: 
            color = strip.Color(255, 0, 0);
            break;
            
        case LED_RED_FADE_OUT: {
            unsigned long elapsed = currentMillis - startTime[i];
            
            int R_fade;
            R_fade = map(elapsed, 0, FADE_OUT_DURATION, 255, 0);
            R_fade = constrain(R_fade, 0, 255);
            
            int B_fade; 
            B_fade = map(elapsed, 0, FADE_OUT_DURATION, 0, 255);
            B_fade = constrain(B_fade, 0, 255); 
            
            color = strip.Color(R_fade, 0, B_fade);
            break;
        }
        
        case LED_RED_MANUAL_FADE_OUT: {
            unsigned long elapsed = currentMillis - startTime[i];
            
            int R_fade;
            R_fade = map(elapsed, 0, FADE_OUT_DURATION, 255, 0);
            R_fade = constrain(R_fade, 0, 255);
            
            color = strip.Color(R_fade, 0, 0);
            break;
        }
            
        case LED_GREEN: {
            unsigned long elapsed = currentMillis - startTime[i];
            int G_brightness;
            G_brightness = map(elapsed, 0, FINISH_TIME, 0, 255);
            G_brightness = constrain(G_brightness, 0, 255);
            color = strip.Color(0, G_brightness, 0);
            break;
        }
        
        case LED_GREEN_FADE_OUT: {
            unsigned long elapsed = currentMillis - startTime[i];
            
            int G_fade;
            G_fade = map(elapsed, 0, FADE_OUT_DURATION, 255, 0);
            G_fade = constrain(G_fade, 0, 255);
            
            color = strip.Color(0, G_fade, 0);
            break;
        }
        
        case LED_BLUE_FLASH: {
            int idx = (i % 2 == 0) ? currentPatternIndexA : currentPatternIndexB;
            if (BLAULICHT_PATTERN[idx] == 1) color = strip.Color(0, 0, 255);
            break;
        }
        case LED_OFF:
        default:
            color = strip.Color(0, 0, 0);
            break;
    }
    
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void handleBlueFlashTimer(unsigned long currentMillis) {
  if (currentMillis - lastPatternChange >= PULSE_DURATION) {
    lastPatternChange = currentMillis;
    currentPatternIndexA = (currentPatternIndexA + 1) % PATTERN_STEPS;
    currentPatternIndexB = (currentPatternIndexB + 1) % PATTERN_STEPS;
  }
}

// ===============================================================================
// 6. HAUPTPROGRAMM (SETUP & LOOP) 
// ===============================================================================

void setup() {
  Serial.begin(115200);
  loadConfiguration();

  // WLAN Access Point starten
  WiFi.softAP(ap_ssid, ap_password, 6, 0, 1);
  server.on("/", [](){ server.send(200, "text/html", generateConfigPage()); });
  server.on("/save", handleSave);
  server.on("/test", handleTestMove);
  server.begin();

  // Hardware Pins konfigurieren
  for (int i = 0; i < 6; i++) pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  pinMode(POTI_PIN, INPUT);
  ledcAttach(PinENA, 5000, 8);
  pinMode(PinIN1, OUTPUT); pinMode(PinIN2, OUTPUT);
  digitalWrite(PinIN1, LOW); digitalWrite(PinIN2, LOW);

  strip.begin(); strip.show();
  ESP32PWM::allocateTimer(1);
  
  // Servos anbinden
  servoLift.attach(SERVO_LIFT_PIN); 
  servoRotate.attach(SERVO_ROTATE_PIN);
  
  // Startwerte für Ist-Position setzen 
  actualLiftUS = liftDownMicroSec; 
  actualRotateUS = rotationMicroSecs[REST_POSITION_INDEX];

  // Sanftes Anfahren der Ruheposition im Setup mit LIFT_UP starten
  currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
  restState = REST_LIFT_UP; 
  
  lastActivityTime = millis();
  globalPulseStartTime = millis(); 
}

void loop() {
  unsigned long currentMillis = millis();
  server.handleClient();
  handleBlueFlashTimer(currentMillis);
  
  handleRestingLED(currentMillis);
  
  long dur = map(analogRead(POTI_PIN), 0, 4095, minFillingTime, maxFillingTime);

  // 1. Alle Sensoren prüfen
  for (int i = 0; i < 6; i++) handleSensorLogic(i, currentMillis);

  // 2. Prüfen, ob ein neues Glas bedient werden muss
  if (!isSystemBusy && testTargetPosition == -1) { // Nur starten, wenn nicht im Testmodus
    for (int i = 0; i < 6; i++) {
      if (ledState[i] == LED_BLUE_FLASH && processState[i] == PROCESS_CONFIRMED) {
        currentProcessingPosition = i; 
        isSystemBusy = true; 
        break;
      }
    }
  }

  // 3. Den aktiven Füllprozess abarbeiten
  if (currentProcessingPosition != -1) {
    handleFillingProcess(currentProcessingPosition, currentMillis, dur);
  }

  // 4. Inaktive Phasen und Servo-Rückkehr/Testbewegungen verwalten
  handleRestingTimeout(currentMillis);
  
  // 5. LEDs aktualisieren
  updateNeoPixels(currentMillis);
}