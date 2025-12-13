//
//                        LOESCHMEISTER ESP32
//                           vom 06.12.25
//
//                         von Stefan Hagel
//                          golfi54@web.de
//

//================================================================================
// BIBLIOTHEKEN EINBINDEN
//================================================================================
#include <Adafruit_NeoPixel.h> 
#include <ESP32Servo.h> // Für die präzise Steuerung der Servomotoren auf dem ESP32
#include <WiFi.h> 
#include <WebServer.h> 
#include <Preferences.h> 

//================================================================================
// HARDWARE-DEFINITIONEN UND KONSTANTEN
//================================================================================
#define LED_PIN 18 
#define SERVO_LIFT_PIN 17 
#define SERVO_ROTATE_PIN 16 
#define POTI_PIN 34 

// WICHTIGE KONSTANTEN FÜR DIE LOGIK
const int NUM_GLAS_POSITIONS = 6; 
const int NUM_TOTAL_POSITIONS = 7; 
const int REST_POSITION_INDEX = 6; 
const int NUM_PIXELS = 6; 

// Pins der 6 IR-Näherungssensoren
const int SENSOR_PINS[NUM_GLAS_POSITIONS] = {32, 33, 25, 26, 27, 14};

// Konstanten für die Zeitsicherheit
const unsigned long DELAY_500MS = 500; 
const unsigned long DELAY_START_PROCESS = 2000; 
const unsigned long MIN_FILLING_TIME = 500; 
const unsigned long FINISH_TIME = 1000; 
const unsigned long MOVEMENT_TIME = 1000; 

// BLAULICHT-MUSTER
const int BLAULICHT_PATTERN[] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, -1};
const long PULSE_DURATION = 55; 
const int PATTERN_STEPS = 17; 
const int STAGGER_OFFSET = PATTERN_STEPS / 2; 

// PUMPE (Motortreiber L298N)
const byte PinENA = 23; 
const byte PinIN1 = 22; 
const byte PinIN2 = 21; 

//================================================================================
// GLOBALE VARIABLEN (WERDEN GESPEICHERT UND SIND IM WEB ÄNDERBAR)
//================================================================================
Preferences preferences; 
WebServer server(80);

// WLAN & WEBSERVER KONFIGURATION (Access Point)
const char* ap_ssid = "Loeschmeister_Konfig"; 
const char* ap_password = "Passwort123"; 

// EINSTELLBARE MIKROSEKUNDEN-PULSZEITEN (μs)
int liftDownMicroSec = 500; 
int liftUpMicroSec = 1500; 
int rotationMicroSecs[NUM_TOTAL_POSITIONS] = {500, 800, 1100, 1400, 1700, 2000, 2300}; 

// EINSTELLBARE GESCHWINDIGKEITEN & ZEITEN
int microSecStep = 5; // Mikrosekunden-Schritt pro Bewegungstakt (Sanftheit)
int stepDelayMs = 15; 
int restDelayMs = 10000; 
long minFillingTime = 500; 
long maxFillingTime = 5000; 
int pumpSpeed = 200; 

//================================================================================
// ZUSTANDSDEFINITIONEN UND GLOBALE STEUERUNGSVARIABLEN
//================================================================================
enum LED_STATE { LED_OFF = 0, LED_ACCEPTED = 1, LED_RED = 2, LED_BLUE_FLASH = 3, LED_GREEN = 4 };
enum FILLING_STATE { PROCESS_IDLE = 0, PROCESS_CONFIRMED = 1, PROCESS_LIFT_UP = 2, PROCESS_ROTATE = 3, PROCESS_LIFT_DOWN = 4, PROCESS_PUMP_ON = 5, PROCESS_PUMP_OFF = 6, PROCESS_COMPLETE = 7, PROCESS_RETURN_LIFT_UP = 8, PROCESS_RETURN_ROTATE = 9, PROCESS_RETURN_LIFT_DOWN = 10 };
enum REST_RETURN_STATE { REST_IDLE = 0, REST_LIFT_UP = 1, REST_ROTATE = 2, REST_LIFT_DOWN = 3 };

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo servoLift;
Servo servoRotate;

LED_STATE ledState[NUM_GLAS_POSITIONS] = {LED_OFF};
FILLING_STATE processState[NUM_GLAS_POSITIONS] = {PROCESS_IDLE};
unsigned long startTime[NUM_GLAS_POSITIONS] = {0};

long currentLiftAngle[NUM_GLAS_POSITIONS];
long currentRotateAngle[NUM_GLAS_POSITIONS];

unsigned long lastPatternChange = 0;
int currentPatternIndexA = 0;
int currentPatternIndexB = STAGGER_OFFSET;

long restModeCurrentLiftAngle;
long restModeCurrentRotateAngle;
unsigned long restProcessStartTime = 0; 

int currentProcessingPosition = -1;
bool isSystemBusy = false;

unsigned long lastActivityTime = 0;
unsigned long lastServoStepTime = 0; 
REST_RETURN_STATE restState = REST_IDLE;

int currentMechanismTargetAngle = -1; 


//================================================================================
// VORWÄRTSDEKLARATIONEN (WICHTIG FÜR DEN COMPILER)
//================================================================================

// Webserver und NVS
void loadConfiguration();
void saveConfiguration();
String generateConfigPage();
void handleRoot();
void handleSave();
void handleTestMove(); 

// Hauptlogik Funktionen
long readPotiValue();
void handleBlueFlashTimer(unsigned long currentMillis);
bool moveServoGradually(Servo &servo, long &currentAngle, long targetAngle, unsigned long currentMillis);
bool moveServoGraduallyRest(Servo &servo, long &currentAngle, long targetAngle, unsigned long currentMillis);
void handleSensorLogic(int position, unsigned long currentMillis);
void handleFillingProcess(int position, unsigned long currentMillis, long fillingDuration);
void handleRestingTimeout(unsigned long currentMillis);
void updateNeoPixels(unsigned long currentMillis);


//================================================================================
// KONFIGURATION SPEICHERN UND LADEN (NVS)
//================================================================================
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
    sprintf(key, "rotUS%d", i);
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
    sprintf(key, "rotUS%d", i);
    preferences.putUInt(key, rotationMicroSecs[i]);
  }
  
  preferences.end();
}


//================================================================================
// WEBSERVER HANDLER IMPLEMENTIERUNGEN
//================================================================================
String generateConfigPage() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><title>Loeschmeister Konfiguration</title>";
  html += "<style>body{font-family:Arial;}label{display:inline-block;width:200px;margin-top:10px;}input[type=number]{width:100px;}.test-btn{padding: 5px 10px; margin-left: 20px; cursor: pointer; background-color: #3f51b5; color: white; border: none; text-decoration: none;}</style>";
  html += "</head><body><h1>Loeschmeister Konfiguration</h1>";

  // --- FORMULAR FÜR DIE KONFIGURATION ---
  html += "<form action='/save' method='post'>";
  
  html += "<h2>1. Servo-Einstellungen (Pulsbreite in Mikrosekunden μs)</h2>";
  html += "<label>Leiter Ab (μs, z.B. 500):</label><input type='number' name='liftDown' value='" + String(liftDownMicroSec) + "'><br>";
  html += "<label>Leiter Auf (μs, z.B. 1500):</label><input type='number' name='liftUp' value='" + String(liftUpMicroSec) + "'><br>";
  html += "<label>Servoverlangsamung (μs-Schritt):</label><input type='number' name='microStep' value='" + String(microSecStep) + "'><br>";
  html += "<label>Schritt-Verzögerung (ms):</label><input type='number' name='stepDelay' value='" + String(stepDelayMs) + "'><br>";
  html += "<label>Ruherueckkehr Wartezeit (ms):</label><input type='number' name='restDelay' value='" + String(restDelayMs) + "'><br>";

  html += "<h2>2. Drehpositionen (Pulsbreite in Mikrosekunden μs) und Test</h2>";
  
  // --- TEST-FUNKTION FÜR JEDE POSITION ---
  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    String label = (i < 6) ? ("Glas " + String(i + 1) + " μs:") : "Ruheposition μs (Pos 7):";
    html += "<label>" + label + "</label>";
    html += "<input type='number' name='rot" + String(i) + "' value='" + String(rotationMicroSecs[i]) + "'>";
    
    // Test-Button für jede Position.
    html += "<a href='/test?pos=" + String(i) + "' class='test-btn' target='_blank'>Test Position " + String(i + 1) + "</a><br>";
  }

  html += "<h2>3. Pumpe & Fuellzeiten</h2>";
  html += "<label>Pumpen-Geschw. (0-255):</label><input type='number' name='pumpSpeed' value='" + String(pumpSpeed) + "'><br>";
  html += "<label>Max. Fuellzeit (ms):</label><input type='number' name='maxFill' value='" + String(maxFillingTime) + "'><br>";
  html += "<input type='submit' value='Einstellungen Speichern' style='margin-top:20px; padding:10px;'>";
  html += "</form></body></html>";
  return html;
}

void handleRoot() {
  server.send(200, "text/html", generateConfigPage());
}

void handleSave() {
  if (server.hasArg("liftDown")) liftDownMicroSec = server.arg("liftDown").toInt();
  if (server.hasArg("liftUp")) liftUpMicroSec = server.arg("liftUp").toInt();
  if (server.hasArg("stepDelay")) stepDelayMs = server.arg("stepDelay").toInt();
  if (server.hasArg("microStep")) microSecStep = server.arg("microStep").toInt();
  
  if (server.hasArg("restDelay")) restDelayMs = server.arg("restDelay").toInt();
  if (server.hasArg("pumpSpeed")) pumpSpeed = server.arg("pumpSpeed").toInt();
  if (server.hasArg("maxFill")) maxFillingTime = server.arg("maxFill").toInt();

  for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
    char argName[10];
    sprintf(argName, "rot%d", i);
    if (server.hasArg(argName)) {
      rotationMicroSecs[i] = server.arg(argName).toInt();
    }
  }
  
  // Sicherheitsbegrenzung
  liftDownMicroSec = constrain(liftDownMicroSec, 400, 2600);
  liftUpMicroSec = constrain(liftUpMicroSec, 400, 2600);
  microSecStep = constrain(microSecStep, 1, 50);
  stepDelayMs = constrain(stepDelayMs, 1, 100);
  pumpSpeed = constrain(pumpSpeed, 0, 255); 
  
  saveConfiguration(); 
  
  String response = "Einstellungen gespeichert! Das Geraet startet neu, um die neuen Pulsbreiten anzuwenden. <meta http-equiv='refresh' content='5; url=/'><p>Startet in 5 Sekunden neu...</p>";
  server.send(200, "text/html", response);
  
  ESP.restart(); 
}

void handleTestMove() {
  if (server.hasArg("pos")) {
    int positionIndex = server.arg("pos").toInt();
    
    if (positionIndex >= 0 && positionIndex < NUM_TOTAL_POSITIONS) {
      int targetMicroSec = rotationMicroSecs[positionIndex];
      
      // 1. Servos anbinden
      if (!servoLift.attached()) {
        servoLift.attach(SERVO_LIFT_PIN);
      }
      if (!servoRotate.attached()) {
        servoRotate.attach(SERVO_ROTATE_PIN);
      }
      
      // 2. Bewegung: Leiter hoch
      servoLift.writeMicroseconds(liftUpMicroSec);
      delay(MOVEMENT_TIME); 
      
      // 3. Bewegung: Rotieren zur Zielposition (im μs-Wert)
      servoRotate.writeMicroseconds(targetMicroSec);
      delay(MOVEMENT_TIME); 

      // 4. Bewegung: Leiter runter
      servoLift.writeMicroseconds(liftDownMicroSec);
      delay(MOVEMENT_TIME);
      
      // 5. Servos wieder trennen (spart Strom und vermeidet Zittern)
      servoLift.detach();
      servoRotate.detach();

      String response = "Position " + String(positionIndex + 1) + " (" + String(targetMicroSec) + " μs) angefahren. Schließe dieses Fenster.";
      server.send(200, "text/plain", response);
      return;
    }
  }

  server.send(400, "text/plain", "Ungueltige Position oder Parameter fehlt.");
}


//================================================================================
// HILFSFUNKTIONEN 
//================================================================================
long readPotiValue() {
  int potiValue = analogRead(POTI_PIN); 
  long scaledTime = map(potiValue, 0, 4095, minFillingTime, maxFillingTime);
  return scaledTime;
}

void handleBlueFlashTimer(unsigned long currentMillis) {
  if (currentMillis - lastPatternChange >= PULSE_DURATION) {
    lastPatternChange = currentMillis;
    
    currentPatternIndexA = (currentPatternIndexA + 1) % PATTERN_STEPS;
    currentPatternIndexB = (currentPatternIndexB + 1) % PATTERN_STEPS;
  }
}

bool moveServoGradually(Servo &servo, long &currentAngle, long targetAngle, unsigned long currentMillis) {
  if (currentAngle == targetAngle) {
    return true; 
  }

  if (currentMillis - lastServoStepTime >= stepDelayMs) {
    lastServoStepTime = currentMillis; 

    if (currentAngle < targetAngle) {
      currentAngle += microSecStep; 
    } else {
      currentAngle -= microSecStep; 
    }

    if (abs(targetAngle - currentAngle) < microSecStep) {
      currentAngle = targetAngle;
    }

    servo.writeMicroseconds(currentAngle);

    Serial.print("GRADUAL MOVE | Servo (μs): ");
    Serial.print(servo.readMicroseconds());
    Serial.print(" | Current (μs): ");
    Serial.print(currentAngle);
    Serial.print(" | Target (μs): ");
    Serial.println(targetAngle);
    
    if (currentAngle == targetAngle) {
        return true; 
    }
  }
  return false; 
}

bool moveServoGraduallyRest(Servo &servo, long &currentAngle, long targetAngle, unsigned long currentMillis) {
  extern unsigned long restProcessStartTime; 
  
  if (currentAngle == targetAngle) {
    return true; 
  }

  if (currentMillis - restProcessStartTime >= stepDelayMs) {
    restProcessStartTime = currentMillis; 

    if (currentAngle < targetAngle) {
      currentAngle += microSecStep; 
    } else {
      currentAngle -= microSecStep; 
    }

    if (abs(targetAngle - currentAngle) < microSecStep) {
      currentAngle = targetAngle;
    }

    servo.writeMicroseconds(currentAngle);
  }
  return false; 
}

//================================================================================
// ZUSTANDSFUNKTIONEN 
//================================================================================
void handleSensorLogic(int position, unsigned long currentMillis) {
  // isGlassPresent ist nur true, wenn der Sensor LOW meldet (invertiert)
  bool isGlassPresent = (digitalRead(SENSOR_PINS[position]) == LOW); 
  
  bool shouldReset = !isGlassPresent && (ledState[position] != LED_OFF);

  if (shouldReset) {
    if (processState[position] == PROCESS_PUMP_ON) {
      ledcWrite(PinENA, 0); 
    }
    
    ledState[position] = LED_OFF;
    processState[position] = PROCESS_IDLE;
    
    if (currentProcessingPosition == position) {
      if (!isSystemBusy) { 
        currentProcessingPosition = -1;
      }
    }
    lastActivityTime = currentMillis; 
    return;
  }

  switch (ledState[position]) {

    case LED_OFF:
      if (isGlassPresent) {
        ledState[position] = LED_ACCEPTED; 
        startTime[position] = currentMillis; 
      }
      break;

    case LED_ACCEPTED: 
      if (currentMillis - startTime[position] >= DELAY_500MS) {
        ledState[position] = LED_RED; 
        startTime[position] = currentMillis; 
        lastActivityTime = currentMillis; 
      }
      break;
      
    case LED_RED: 
      if (currentMillis - startTime[position] >= DELAY_START_PROCESS) {
        ledState[position] = LED_BLUE_FLASH; 
        processState[position] = PROCESS_CONFIRMED;
        lastActivityTime = currentMillis; 
      }
      break;

    case LED_BLUE_FLASH:
    case LED_GREEN:
      lastActivityTime = currentMillis;
      break;
  }
}


void handleFillingProcess(int position, unsigned long currentMillis, long fillingDuration) {
  
  if (position != currentProcessingPosition) {
    return;
  }
  
  switch (processState[position]) {
    
    case PROCESS_CONFIRMED:
      if (currentMillis - startTime[position] >= 100) { 
        
        // Servos anbinden
        if (!servoLift.attached()) {
          servoLift.attach(SERVO_LIFT_PIN);
        }
        if (!servoRotate.attached()) {
          servoRotate.attach(SERVO_ROTATE_PIN);
        }

        processState[position] = PROCESS_LIFT_UP;
        // WICHTIG: Startposition ist der global gespeicherte Wert
        currentLiftAngle[position] = liftDownMicroSec; 
        startTime[position] = currentMillis; 
      }
      break;
      
    case PROCESS_LIFT_UP:
    
      if (moveServoGradually(servoLift, currentLiftAngle[position], liftUpMicroSec, currentMillis)) {
        processState[position] = PROCESS_ROTATE;
        // KORREKTUR: Startrotation vom gespeicherten Ruhewert
        currentRotateAngle[position] = rotationMicroSecs[REST_POSITION_INDEX]; 
        startTime[position] = currentMillis; 
      }
      break;

    case PROCESS_ROTATE:
      
      if (moveServoGradually(servoRotate, currentRotateAngle[position], rotationMicroSecs[position], currentMillis)) {
        processState[position] = PROCESS_LIFT_DOWN;
        startTime[position] = currentMillis; 
      }
      break;

    case PROCESS_LIFT_DOWN:

      if (moveServoGradually(servoLift, currentLiftAngle[position], liftDownMicroSec, currentMillis)) {
        processState[position] = PROCESS_PUMP_ON;
        
        digitalWrite(PinIN1, HIGH);   
        digitalWrite(PinIN2, LOW);   
        ledcWrite(PinENA, pumpSpeed); 

        startTime[position] = currentMillis; 
      }
      break;

    case PROCESS_PUMP_ON:
      if (currentMillis - startTime[position] >= fillingDuration) {
        processState[position] = PROCESS_PUMP_OFF;

        ledcWrite(PinENA, 0); 

        ledState[position] = LED_GREEN; 
        startTime[position] = currentMillis; 
      }
      break;
      
    case PROCESS_PUMP_OFF:
      if (currentMillis - startTime[position] >= FINISH_TIME) {
        
        int nextGlass = -1;
        for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
          if (processState[i] == PROCESS_CONFIRMED) { 
            nextGlass = i;
            break;
          }
        }
        
        if (nextGlass != -1) {
          currentMechanismTargetAngle = rotationMicroSecs[nextGlass];
          currentProcessingPosition = nextGlass;
        } else {
          currentMechanismTargetAngle = rotationMicroSecs[REST_POSITION_INDEX];
        }

        processState[position] = PROCESS_RETURN_LIFT_UP; 
        // WICHTIG: Startposition vom aktuell erreichten Wert
        currentLiftAngle[position] = liftDownMicroSec; 
        startTime[position] = currentMillis; 
      }
      break;

    case PROCESS_RETURN_LIFT_UP:
      if (moveServoGradually(servoLift, currentLiftAngle[position], liftUpMicroSec, currentMillis)) {
        processState[position] = PROCESS_RETURN_ROTATE;
        // KORREKTUR: Startrotation von der zuletzt angefahrenen Glasposition
        currentRotateAngle[position] = rotationMicroSecs[position]; 
        startTime[position] = currentMillis; 
      }
      break;

    case PROCESS_RETURN_ROTATE:
      if (moveServoGradually(servoRotate, currentRotateAngle[position], currentMechanismTargetAngle, currentMillis)) {
        
        if (currentMechanismTargetAngle == rotationMicroSecs[REST_POSITION_INDEX]) {
          processState[position] = PROCESS_RETURN_LIFT_DOWN;
        } else {
          
          // Übergabe an den nächsten Prozess
          processState[position] = PROCESS_COMPLETE; 
          
          // Der neue Prozess (nextGlass) kann nun ab dem LIFT_DOWN-Schritt beginnen
          currentLiftAngle[currentProcessingPosition] = liftUpMicroSec; 
          processState[currentProcessingPosition] = PROCESS_LIFT_DOWN;
        }
        startTime[position] = currentMillis; 
      }
      break;
      
    case PROCESS_RETURN_LIFT_DOWN:
      if (moveServoGradually(servoLift, currentLiftAngle[position], liftDownMicroSec, currentMillis)) {
        
        processState[position] = PROCESS_COMPLETE; 
        
        isSystemBusy = false;       
        currentProcessingPosition = -1;     
        
        servoLift.detach();
        servoRotate.detach();
      }
      break;
      
    case PROCESS_COMPLETE:
      break;
  }
}

void handleRestingTimeout(unsigned long currentMillis) {
  if (isSystemBusy) {
    restState = REST_IDLE;
    return;
  }
  
  if (restState == REST_IDLE) {
    bool isLiftRest = (restModeCurrentLiftAngle == liftDownMicroSec);
    bool isRotateRest = (restModeCurrentRotateAngle == rotationMicroSecs[REST_POSITION_INDEX]);

    if (!isLiftRest || !isRotateRest) {
      if (currentMillis - lastActivityTime >= restDelayMs) { 
        
        if (!servoLift.attached()) {
          servoLift.attach(SERVO_LIFT_PIN);
        }
        if (!servoRotate.attached()) {
          servoRotate.attach(SERVO_ROTATE_PIN);
        }
        
        restState = REST_LIFT_UP; 
        restProcessStartTime = currentMillis; 
      }
    }
  }

  switch (restState) {
    
    case REST_LIFT_UP:
      // Start von der gespeicherten Position zur Auf-Position
      if (moveServoGraduallyRest(servoLift, restModeCurrentLiftAngle, liftUpMicroSec, currentMillis)) {
        restState = REST_ROTATE;
        // KORREKTUR: Startrotation von der gespeicherten Position
        restModeCurrentRotateAngle = restModeCurrentRotateAngle; 
        restProcessStartTime = currentMillis; 
      }
      break;

    case REST_ROTATE:
      if (moveServoGraduallyRest(servoRotate, restModeCurrentRotateAngle, rotationMicroSecs[REST_POSITION_INDEX], currentMillis)) {
        restState = REST_LIFT_DOWN;
        restProcessStartTime = currentMillis; 
      }
      break;

    case REST_LIFT_DOWN:
      if (moveServoGraduallyRest(servoLift, restModeCurrentLiftAngle, liftDownMicroSec, currentMillis)) {
        restState = REST_IDLE; 
        
        servoLift.detach();
        servoRotate.detach();
      }
      break;

    default:
      break;
  }
}

void updateNeoPixels(unsigned long currentMillis) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    uint32_t color = strip.Color(0, 0, 0);

    switch (ledState[i]) {
      
      case LED_ACCEPTED: 
        color = strip.Color(100, 0, 0); 
        break;

      case LED_RED: 
        color = strip.Color(255, 0, 0); 
        break;

      case LED_GREEN:
        color = strip.Color(0, 255, 0); 
        break;

      case LED_BLUE_FLASH:
        {
          int patternIndexToUse = (i % 2 == 0) ? currentPatternIndexA : currentPatternIndexB;
          
          if (BLAULICHT_PATTERN[patternIndexToUse] == 1) {
            color = strip.Color(0, 0, 255); 
          }
        }
        break;

      case LED_OFF:
      default:
        break;
    }

    strip.setPixelColor(i, color);
  }
  strip.show(); 
}


//================================================================================
// SETUP 
//================================================================================
void setup() {
  Serial.begin(115200);
  
  // 1. Konfiguration laden
  loadConfiguration();

  // 2. WLAN Access Point starten
  WiFi.softAP(ap_ssid, ap_password, 6, 0, 1); 
  Serial.print("Access Point gestartet. IP: ");
  Serial.println(WiFi.softAPIP());

  // 3. Webserver initialisieren
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.on("/test", handleTestMove); 
  server.begin();

  // 4. Hardware Initialisierung
  for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  }
  pinMode(POTI_PIN, INPUT); 
  
  ledcAttach(PinENA, 5000, 8); 
  
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  digitalWrite(PinIN1, LOW); 
  digitalWrite(PinIN2, LOW);

  strip.begin();
  strip.setBrightness(60);
  strip.show();

  // Servos initialisieren
  servoLift.attach(SERVO_LIFT_PIN);
  servoRotate.attach(SERVO_ROTATE_PIN);
  
  // 5. KRITISCHE SICHERHEITSSEQUENZ BEIM START (BLOCKIEREND!)
  servoLift.writeMicroseconds(liftUpMicroSec); 
  delay(MOVEMENT_TIME); 
  servoRotate.writeMicroseconds(rotationMicroSecs[REST_POSITION_INDEX]);
  delay(MOVEMENT_TIME); 
  servoLift.writeMicroseconds(liftDownMicroSec);
  delay(MOVEMENT_TIME);
  
  // Servos initial trennen
  servoLift.detach();
  servoRotate.detach();
  
  // WICHTIGE KORREKTUR: Initialisiere die globalen μs-Variablen nach der Start-Sequenz
  for(int i=0; i < NUM_GLAS_POSITIONS; i++) {
    currentLiftAngle[i] = liftDownMicroSec; 
    currentRotateAngle[i] = rotationMicroSecs[REST_POSITION_INDEX];
    processState[i] = PROCESS_IDLE; 
  }
  // Initialisiere die dedizierten Ruhezustands-Variablen
  restModeCurrentLiftAngle = liftDownMicroSec;
  restModeCurrentRotateAngle = rotationMicroSecs[REST_POSITION_INDEX];

  lastActivityTime = millis(); 
}

//================================================================================
// LOOP
//================================================================================
void loop() {
  unsigned long currentMillis = millis();

  server.handleClient(); 

  // 1. Blaulicht-Taktgeber 
  handleBlueFlashTimer(currentMillis);
  
  // 2. Pumpenzeit aus Poti lesen
  long fillingDuration = readPotiValue();

  // 3. Sensor-Logik für alle Positionen
  for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
    handleSensorLogic(i, currentMillis);
  }

  // 4. Prozess-Arbitration (Startet die Sequenz)
  if (!isSystemBusy) {
    for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
      if (ledState[i] == LED_BLUE_FLASH && processState[i] == PROCESS_CONFIRMED) {
        currentProcessingPosition = i; 
        isSystemBusy = true;      
        
        // Servos anhängen (Wird im Process_CONFIRMED Schritt des Handlers erneut geprüft)
        
        break;         
      }
    }
  }

  // 5. Nur den EINEN AKTIVEN Prozess abarbeiten
  if (currentProcessingPosition != -1) {
    handleFillingProcess(currentProcessingPosition, currentMillis, fillingDuration);
  }

  // 6. Automatische Ruherückkehr bei Inaktivität
  handleRestingTimeout(currentMillis);

  // 7. LEDs aktualisieren
  updateNeoPixels(currentMillis);
}