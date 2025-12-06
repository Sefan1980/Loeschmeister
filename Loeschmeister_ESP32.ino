#include <Adafruit_NeoPixel.h>
#include <ESP32-Servo.h>

// --- HARDWARE-KONFIGURATION ---
#define LED_PIN 18      // Datenpin des NeoPixel-Streifens
#define PUMP_PIN 23     // Pin für die Pumpe
#define SERVO_LIFT_PIN 17   // Pin für den Hub-Servo
#define SERVO_ROTATE_PIN 16 // Pin für den Dreh-Servo
#define POTI_PIN 34     // Analoger Pin für das Potentiometer (Pumpenzeit)

const int NUM_GLAS_POSITIONS = 6;
const int NUM_TOTAL_POSITIONS = 7; // 6 Gläser + 1 Ruheposition (Index 6)
const int NUM_PIXELS = 6;
const int SENSOR_PINS[NUM_GLAS_POSITIONS] = {32, 33, 25, 26, 27, 14};

// --- SERVO WINKEL & BEWEGUNG ---
const int LIFT_DOWN_ANGLE = 20;  
const int LIFT_UP_ANGLE = 160;  
const int ROTATION_ANGLES[NUM_TOTAL_POSITIONS] = {0, 30, 60, 90, 120, 150, 180}; 
const int REST_POSITION_INDEX = 6;

// Für die blockierende Startsequenz:
const unsigned long MOVEMENT_TIME = 1000;    // Wartezeit für volle Bewegung in Setup()

// --- GESCHWINDIGKEITSKONFIGURATION (für nicht-blockierende Bewegung) ---
const int STEP_DELAY_MS = 15; // Zeit zwischen den Grad-Schritten (je höher, desto langsamer)
const int ANGLE_STEP = 1;     // Grad-Inkrement pro Schritt

// --- ZEIT-KONSTANTEN ---
const unsigned long DELAY_500MS = 500;
const unsigned long MIN_FILLING_TIME = 500;
const unsigned long MAX_FILLING_TIME = 5000;
const unsigned long FINISH_TIME = 1000;

// --- BLAULICHT-MUSTER ---
const int BLAULICHT_PATTERN[] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, -1};
const long PULSE_DURATION = 117; 
const int PATTERN_STEPS = 17;    
const int STAGGER_OFFSET = PATTERN_STEPS / 2;

// --- ZUSTANDSDEFINITIONEN ---
enum LED_STATE { LED_OFF = 0, LED_RED = 1, LED_BLUE_FLASH = 2, LED_GREEN = 3 };

enum FILLING_STATE {
  PROCESS_IDLE = 0,
  PROCESS_CONFIRMED = 1,
  PROCESS_LIFT_UP = 2,
  PROCESS_ROTATE = 3,
  PROCESS_LIFT_DOWN = 4,
  PROCESS_PUMP_ON = 5,
  PROCESS_PUMP_OFF = 6,
  PROCESS_COMPLETE = 7,
  PROCESS_RETURN_LIFT_UP = 8,
  PROCESS_RETURN_ROTATE = 9,
  PROCESS_RETURN_LIFT_DOWN = 10
};


// --- GLOBALE VARIABLEN ---
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo servoLift;
Servo servoRotate;

LED_STATE ledState[NUM_GLAS_POSITIONS] = {LED_OFF};
FILLING_STATE processState[NUM_GLAS_POSITIONS] = {PROCESS_IDLE};
unsigned long startTime[NUM_GLAS_POSITIONS] = {0};

// Winkel-Speicher für die nicht-blockierende Bewegung
int currentLiftAngle[NUM_GLAS_POSITIONS];
int currentRotateAngle[NUM_GLAS_POSITIONS];

// Blaulicht-Timing
unsigned long lastPatternChange = 0;
int currentPatternIndexA = 0;
int currentPatternIndexB = STAGGER_OFFSET;

// Warteschlangen-Steuerung
int currentProcessingPosition = -1; // -1: System ist im Leerlauf
bool isSystemBusy = false;

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  }
  pinMode(POTI_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  strip.begin();
  strip.setBrightness(60);
  strip.show();

  servoLift.attach(SERVO_LIFT_PIN);
  servoRotate.attach(SERVO_ROTATE_PIN);
  
  // --- KRITISCHE SICHERHEITSSEQUENZ BEIM START (BLOCKIEREND!) ---
  servoLift.write(LIFT_UP_ANGLE); 
  delay(MOVEMENT_TIME); 
  servoRotate.write(ROTATION_ANGLES[REST_POSITION_INDEX]);
  delay(MOVEMENT_TIME); 
  servoLift.write(LIFT_DOWN_ANGLE);
  delay(MOVEMENT_TIME); 
  
  // Initialisierung der globalen Winkel-Variablen
  for(int i=0; i < NUM_GLAS_POSITIONS; i++) {
    currentLiftAngle[i] = LIFT_DOWN_ANGLE;
    currentRotateAngle[i] = ROTATION_ANGLES[REST_POSITION_INDEX];
  }
}

// --- HAUPTPROGRAMM-SCHLEIFE ---
void loop() {
  unsigned long currentMillis = millis();

  // 1. Blaulicht-Taktgeber
  handleBlueFlashTimer(currentMillis);
  
  // 2. Pumpenzeit aus Poti lesen
  long fillingDuration = readPotiValue();

  // 3. Sensor-Logik für alle Positionen (erkennt wartende Gläser & Hard-Reset)
  for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
    handleSensorLogic(i, currentMillis);
  }

  // 4. Prozess-Arbitration (Steuert die Warteschlange)
  if (!isSystemBusy) {
    for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
      if (ledState[i] == LED_BLUE_FLASH && processState[i] == PROCESS_CONFIRMED) {
        currentProcessingPosition = i; 
        isSystemBusy = true;           
        break; 
      }
    }
  }

  // 5. Nur den EINEN AKTIVEN Prozess abarbeiten
  if (isSystemBusy) {
    handleFillingProcess(currentProcessingPosition, currentMillis, fillingDuration);
  }

  // 6. LEDs aktualisieren
  updateNeoPixels(currentMillis);
}

// --- NEUE HILFSFUNKTION FÜR NICHT-BLOCKIERENDE SCHRITTWEISE BEWEGUNG ---
/**
 * Bewegt den Servo schrittweise zum Ziel, gibt 'true' zurück, wenn Ziel erreicht.
 * Nutzt den startTime[positionTimerIndex] als Timer für die Schrittfrequenz.
 */
bool moveServoGradually(Servo &servo, int &currentAngle, int targetAngle, unsigned long currentMillis, int positionTimerIndex) {
    if (currentAngle == targetAngle) {
        return true; // Ziel erreicht
    }

    if (currentMillis - startTime[positionTimerIndex] >= STEP_DELAY_MS) {
        startTime[positionTimerIndex] = currentMillis; // Setze neuen Schritt-Timer

        // Bestimme die Richtung der Bewegung
        if (currentAngle < targetAngle) {
            currentAngle += ANGLE_STEP;
        } else {
            currentAngle -= ANGLE_STEP;
        }

        // Stelle sicher, dass der Winkel das Ziel nicht überschreitet, 
        // um das Flackern am Ende der Bewegung zu verhindern
        if (abs(targetAngle - currentAngle) < ANGLE_STEP) {
             currentAngle = targetAngle;
        }

        servo.write(currentAngle);
    }
    return false; // Bewegung läuft noch
}


/**
 * Liest den Potentiometer-Wert und skaliert ihn auf die Füllzeit.
 */
long readPotiValue() {
  int potiValue = analogRead(POTI_PIN); 
  long scaledTime = map(potiValue, 0, 4095, MIN_FILLING_TIME, MAX_FILLING_TIME);
  return scaledTime;
}


/**
 * Steuert den nicht-blockierenden Takt für das Blaulicht-Muster.
 */
void handleBlueFlashTimer(unsigned long currentMillis) {
  if (currentMillis - lastPatternChange >= PULSE_DURATION) {
    lastPatternChange = currentMillis;
    
    currentPatternIndexA = (currentPatternIndexA + 1) % PATTERN_STEPS;
    currentPatternIndexB = (currentPatternIndexB + 1) % PATTERN_STEPS;
  }
}

/**
 * Steuert die Logik der Näherungssensoren und die ersten Zustandsübergänge.
 */
void handleSensorLogic(int position, unsigned long currentMillis) {
  bool isGlassPresent = (digitalRead(SENSOR_PINS[position]) == LOW);

  switch (ledState[position]) {

    case LED_OFF:
      if (isGlassPresent) {
        ledState[position] = LED_RED;
        startTime[position] = currentMillis; 
      }
      break;

    case LED_RED:
      if (!isGlassPresent) {
        // Glas entfernt -> Reset
        ledState[position] = LED_OFF;
        processState[position] = PROCESS_IDLE;
      } else if (currentMillis - startTime[position] >= DELAY_500MS) {
        // 500ms abgelaufen -> Glas bestätigt
        
        // ABER: Starte den Prozess nur, wenn das System NICHT beschäftigt ist, 
        // ODER wenn die aktuelle Position bereits an der Reihe ist.
        if (!isSystemBusy || currentProcessingPosition == position) {
            // Dies markiert die Position als bereit für die Warteschlange
            ledState[position] = LED_BLUE_FLASH; 
            processState[position] = PROCESS_CONFIRMED; 
        }
      }
      break;

    case LED_BLUE_FLASH:
    case LED_GREEN:
      if (!isGlassPresent) {
        // --- HARDWARE-SICHERHEIT BEIM RESET ---
        if (processState[position] == PROCESS_PUMP_ON) {
          digitalWrite(PUMP_PIN, LOW); // Pumpe SOFORT ausschalten!
        }
        
        // Reset des Prozesses
        ledState[position] = LED_OFF;
        processState[position] = PROCESS_IDLE;
        
        // Freigabe des Systems, falls dieses Glas gerade aktiv war
        if (currentProcessingPosition == position) {
             isSystemBusy = false;
             currentProcessingPosition = -1;
        }
      }
      break;
  }
}

/**
 * Steuert die mehrstufige Befüllungssequenz (mit Geschwindigkeitssteuerung).
 */
void handleFillingProcess(int position, unsigned long currentMillis, long fillingDuration) {
  // Wenn das System auf Leerlauf gesetzt wurde (durch Glasentfernung), beenden
  if (processState[position] == PROCESS_IDLE || ledState[position] == LED_OFF) {
    return;
  }

  // Ab hier wird die Servo-Bewegung über moveServoGradually gesteuert.
  
  switch (processState[position]) {
    
    case PROCESS_CONFIRMED:
      // 1. Übergang: Start der Bewegung
      if (currentMillis - startTime[position] >= 100) { 
        processState[position] = PROCESS_LIFT_UP;
        // Startwinkel für die schrittweise Steuerung setzen
        currentLiftAngle[position] = servoLift.read(); 
      }
      break;
      
    case PROCESS_LIFT_UP:
      // 2. Warten, bis Leiter angehoben ist
      if (moveServoGradually(servoLift, currentLiftAngle[position], LIFT_UP_ANGLE, currentMillis, position)) {
        processState[position] = PROCESS_ROTATE;
        currentRotateAngle[position] = servoRotate.read(); 
      }
      break;

    case PROCESS_ROTATE:
      // 3. Warten, bis zur Position gedreht ist
      if (moveServoGradually(servoRotate, currentRotateAngle[position], ROTATION_ANGLES[position], currentMillis, position)) {
        processState[position] = PROCESS_LIFT_DOWN;
      }
      break;

    case PROCESS_LIFT_DOWN:
      // 4. Warten, bis Leiter abgesenkt ist
      if (moveServoGradually(servoLift, currentLiftAngle[position], LIFT_DOWN_ANGLE, currentMillis, position)) {
        processState[position] = PROCESS_PUMP_ON;
        digitalWrite(PUMP_PIN, HIGH); // Pumpe AN
        startTime[position] = currentMillis;
      }
      break;

    case PROCESS_PUMP_ON:
      // 5. Warten auf das Ende der Füllzeit (Dynamisch vom Poti)
      if (currentMillis - startTime[position] >= fillingDuration) {
        processState[position] = PROCESS_PUMP_OFF;
        digitalWrite(PUMP_PIN, LOW); // Pumpe AUS
        ledState[position] = LED_GREEN; 
        startTime[position] = currentMillis; 
      }
      break;
      
    case PROCESS_PUMP_OFF:
      // 6. Warten auf die Anzeigezeit des grünen Lichts
      if (currentMillis - startTime[position] >= FINISH_TIME) {
        processState[position] = PROCESS_RETURN_LIFT_UP; 
        currentLiftAngle[position] = servoLift.read(); 
      }
      break;

    case PROCESS_RETURN_LIFT_UP:
      // 7. Ruherückkehr: Leiter anheben
      if (moveServoGradually(servoLift, currentLiftAngle[position], LIFT_UP_ANGLE, currentMillis, position)) {
        processState[position] = PROCESS_RETURN_ROTATE;
        currentRotateAngle[position] = servoRotate.read(); 
      }
      break;

    case PROCESS_RETURN_ROTATE:
      // 8. Ruherückkehr: Zur Ruheposition drehen
      if (moveServoGradually(servoRotate, currentRotateAngle[position], ROTATION_ANGLES[REST_POSITION_INDEX], currentMillis, position)) {
        processState[position] = PROCESS_RETURN_LIFT_DOWN;
      }
      break;
      
    case PROCESS_RETURN_LIFT_DOWN:
      // 9. Ruherückkehr: Absenken
      if (moveServoGradually(servoLift, currentLiftAngle[position], LIFT_DOWN_ANGLE, currentMillis, position)) {
        processState[position] = PROCESS_COMPLETE; 
        // Hier bleibt das System Busy, bis das Glas entfernt wird!
      }
      break;
      
    case PROCESS_COMPLETE:
      // 10. Warten, bis das Glas entfernt wird (Freigabe des Systems erfolgt in handleSensorLogic).
      break;
  }
}

/**
 * Setzt die Farbe aller LEDs basierend auf ihrem aktuellen Zustand.
 */
void updateNeoPixels(unsigned long currentMillis) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    uint32_t color = strip.Color(0, 0, 0);

    switch (ledState[i]) {
      case LED_RED:
        color = strip.Color(255, 0, 0);
        break;

      case LED_GREEN:
        color = strip.Color(0, 255, 0);
        break;

      case LED_BLUE_FLASH:
        {
          int patternIndexToUse;
          if (i % 2 == 0) { 
            patternIndexToUse = currentPatternIndexA;
          } else { 
            patternIndexToUse = currentPatternIndexB;
          }
          
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
