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
#include <Adafruit_NeoPixel.h> // Für die Steuerung der LED-Streifen (Lichter)
#include <ESP32Servo.h> // Für die präzise Steuerung der Servomotoren auf dem ESP32
#include <WiFi.h> // Für WLAN-Funktionen (Access Point und Webserver)
#include <WebServer.h> // Für den HTTP-Webserver zur Konfiguration
#include <Preferences.h> // Für den nichtflüchtigen Speicher (NVS), um Einstellungen zu speichern

//================================================================================
// HARDWARE-DEFINITIONEN UND KONSTANTEN
//================================================================================
#define LED_PIN 18 // GPIO 18: Datenpin des NeoPixel-Streifens
#define SERVO_LIFT_PIN 17 // GPIO 17: Pin für den Hub-Servo (Leiter rauf/runter)
#define SERVO_ROTATE_PIN 16 // GPIO 16: Pin für den Dreh-Servo (Kreisposition)
#define POTI_PIN 34 // GPIO 34: Analoger Pin für das Potentiometer (Fülldauer)

// WICHTIGE KONSTANTEN FÜR DIE LOGIK
const int NUM_GLAS_POSITIONS = 6; // Es gibt 6 Positionen für Gläser (Index 0 bis 5)
const int NUM_TOTAL_POSITIONS = 7; // Das Winkel-Array hat 7 Elemente (Index 0 bis 6)
const int REST_POSITION_INDEX = 6; // Der feste Index im Winkel-Array, der für die Ruheposition reserviert ist.
const int NUM_PIXELS = 6; // Anzahl der LEDs auf dem Streifen

// Pins der 6 IR-Näherungssensoren
const int SENSOR_PINS[NUM_GLAS_POSITIONS] = {32, 33, 25, 26, 27, 14};

// Konstanten für die Zeitsicherheit
const unsigned long DELAY_500MS = 500; // 0,5 Sekunden: Kurze Wartezeit zur Glasbestätigung
const unsigned long DELAY_START_PROCESS = 2000; // 2,0 Sekunden: Zusätzliche Toleranzzeit (wichtig für "Ruhe")
const unsigned long MIN_FILLING_TIME = 500; // Minimale Füllzeit aus Poti (ms)
const unsigned long FINISH_TIME = 1000; // Grüne LED Leuchtdauer nach Füllung (ms)
const unsigned long MOVEMENT_TIME = 1000; // Wartezeit für blockierende Bewegungen im Setup (Sicherheit)

// BLAULICHT-MUSTER
const int BLAULICHT_PATTERN[] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, -1};
const long PULSE_DURATION = 117; // Zeit für jeden Schritt des Musters (ms)
const int PATTERN_STEPS = 17; // Länge des Musters
const int STAGGER_OFFSET = PATTERN_STEPS / 2; // Versatz, damit die LEDs abwechselnd blinken

// PUMPE (Motortreiber L298N)
const byte PinENA = 23; // GPIO 23: Pin für die Geschw. (PWM)
const byte PinIN1 = 22; // GPIO 27: IN1 Motortreiber (Richtung)
const byte PinIN2 = 21; // GPIO 26: IN2 Motortreiber (Richtung)

//================================================================================
// GLOBALE VARIABLEN (WERDEN GESPEICHERT UND SIND IM WEB ÄNDERBAR)
//================================================================================
Preferences preferences; // Objekt für den nichtflüchtigen Speicher (NVS)
WebServer server(80);

// WLAN & WEBSERVER KONFIGURATION (Access Point)
const char* ap_ssid = "Loeschmeister_Konfig"; 
const char* ap_password = "Passwort123"; // HIER IHR PASSWORT EINGEBEN!

// EINSTELLBARE WINKEL
int liftDownAngle = 20; // Leiter abgesenkt (Ruheposition)
int liftUpAngle = 160; // Leiter angehoben (Voraussetzung für Drehung)
int rotationAngles[NUM_TOTAL_POSITIONS] = {0, 30, 60, 90, 120, 150, 180}; // Winkel für Pos 1-6 und Ruhepos 7

// EINSTELLBARE GESCHWINDIGKEITEN & ZEITEN
int angleStep = 1; // Grad-Schritt pro Bewegungstakt (Sanftheit)
int stepDelayMs = 15; // Zeit zwischen den Grad-Schritten (Geschwindigkeit)
int restDelayMs = 10000; // Zeit bis zur automatischen Ruherückkehr (ms)
long minFillingTime = 500; // Minimum Füllzeit (Poti-Grenze)
long maxFillingTime = 5000; // Maximum Füllzeit (Poti-Grenze)
int pumpSpeed = 200; // Pumpe Geschw. (0-255)

//================================================================================
// ZUSTANDSDEFINITIONEN UND GLOBALE STEUERUNGSVARIABLEN
//================================================================================
enum LED_STATE { LED_OFF = 0, LED_ACCEPTED = 1, LED_RED = 2, LED_BLUE_FLASH = 3, LED_GREEN = 4 };

// Geänderte Zustände: Rückkehrzustände werden jetzt für die Bewegung zum nächsten Glas oder zur Ruheposition verwendet
enum FILLING_STATE { PROCESS_IDLE = 0, PROCESS_CONFIRMED = 1, PROCESS_LIFT_UP = 2, PROCESS_ROTATE = 3, PROCESS_LIFT_DOWN = 4, PROCESS_PUMP_ON = 5, PROCESS_PUMP_OFF = 6, PROCESS_COMPLETE = 7, PROCESS_RETURN_LIFT_UP = 8, PROCESS_RETURN_ROTATE = 9, PROCESS_RETURN_LIFT_DOWN = 10 };

enum REST_RETURN_STATE { REST_IDLE = 0, REST_LIFT_UP = 1, REST_ROTATE = 2, REST_LIFT_DOWN = 3 };

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo servoLift;
Servo servoRotate;

LED_STATE ledState[NUM_GLAS_POSITIONS] = {LED_OFF};
FILLING_STATE processState[NUM_GLAS_POSITIONS] = {PROCESS_IDLE};
unsigned long startTime[NUM_GLAS_POSITIONS] = {0};


int currentLiftAngle[NUM_GLAS_POSITIONS];
int currentRotateAngle[NUM_GLAS_POSITIONS];

unsigned long lastPatternChange = 0;
int currentPatternIndexA = 0;
int currentPatternIndexB = STAGGER_OFFSET;

// DEDIZIERTE VARIABLEN FÜR DEN RUHEZUSTAND (REST-RETURN)
int restModeCurrentLiftAngle;
int restModeCurrentRotateAngle;
unsigned long restProcessStartTime = 0; // Dedizierter Timer für die Ruherückkehr

int currentProcessingPosition = -1;
bool isSystemBusy = false;

unsigned long lastActivityTime = 0;
REST_RETURN_STATE restState = REST_IDLE;

// NEUE GLOBALE STEUERUNGSVARIABLE FÜR SEQUENZIELLES FÜLLEN
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

// Hauptlogik Funktionen
long readPotiValue();
void handleBlueFlashTimer(unsigned long currentMillis);
bool moveServoGradually(Servo &servo, int &currentAngle, int targetAngle, unsigned long currentMillis, int positionTimerIndex);
bool moveServoGraduallyRest(Servo &servo, int &currentAngle, int targetAngle, unsigned long currentMillis);
void handleSensorLogic(int position, unsigned long currentMillis);
void handleFillingProcess(int position, unsigned long currentMillis, long fillingDuration);
void handleRestingTimeout(unsigned long currentMillis);
void updateNeoPixels(unsigned long currentMillis);


//================================================================================
// KONFIGURATION SPEICHERN UND LADEN (NVS)
//================================================================================
void loadConfiguration() {
	preferences.begin("lox-config", true); 

	liftDownAngle = preferences.getUInt("liftDown", liftDownAngle);
	liftUpAngle = preferences.getUInt("liftUp", liftUpAngle);
	stepDelayMs = preferences.getUInt("stepDelay", stepDelayMs);
	angleStep = preferences.getUInt("angleStep", angleStep);
	restDelayMs = preferences.getUInt("restDelay", restDelayMs);
	pumpSpeed = preferences.getUInt("pumpSpeed", pumpSpeed);
	minFillingTime = preferences.getUInt("minFill", minFillingTime);
	maxFillingTime = preferences.getUInt("maxFill", maxFillingTime);

	for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
		char key[10];
		sprintf(key, "rot%d", i);
		rotationAngles[i] = preferences.getUInt(key, rotationAngles[i]);
	}

	preferences.end();
}

void saveConfiguration() {
	preferences.begin("lox-config", false); 

	preferences.putUInt("liftDown", liftDownAngle);
	preferences.putUInt("liftUp", liftUpAngle);
	preferences.putUInt("stepDelay", stepDelayMs);
	preferences.putUInt("angleStep", angleStep);
	preferences.putUInt("restDelay", restDelayMs);
	preferences.putUInt("pumpSpeed", pumpSpeed);
	preferences.putUInt("minFill", minFillingTime);
	preferences.putUInt("maxFill", maxFillingTime);

	for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
		char key[10];
		sprintf(key, "rot%d", i);
		preferences.putUInt(key, rotationAngles[i]);
	}
	
	preferences.end();
}


//================================================================================
// WEBSERVER HANDLER
//================================================================================
String generateConfigPage() {
	String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><title>Loeschmeister Konfiguration</title>";
	html += "<style>body{font-family:Arial;}label{display:inline-block;width:200px;margin-top:10px;}input[type=number]{width:100px;}</style>";
	html += "</head><body><h1>Loeschmeister Konfiguration</h1><form action='/save' method='post'>";
	
	html += "<h2>1. Allgemeine Einstellungen</h2>";
	html += "<label>Leiter Ab (Grad):</label><input type='number' name='liftDown' value='" + String(liftDownAngle) + "'><br>";
	html += "<label>Leiter Auf (Grad):</label><input type='number' name='liftUp' value='" + String(liftUpAngle) + "'><br>";
	html += "<label>Servoverlangsamung (ms):</label><input type='number' name='stepDelay' value='" + String(stepDelayMs) + "'><br>";
	html += "<label>Ruherueckkehr Wartezeit (ms):</label><input type='number' name='restDelay' value='" + String(restDelayMs) + "'><br>";

	html += "<h2>2. Drehwinkel pro Position (0-180 Grad)</h2>";
	for(int i = 0; i < 6; i++) {
		html += "<label>Glas " + String(i + 1) + " Winkel:</label><input type='number' name='rot" + String(i) + "' value='" + String(rotationAngles[i]) + "'><br>";
	}
	html += "<label>Ruheposition Winkel (Pos 7):</label><input type='number' name='rot6' value='" + String(rotationAngles[6]) + "'><br>";

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
	if (server.hasArg("liftDown")) liftDownAngle = server.arg("liftDown").toInt();
	if (server.hasArg("liftUp")) liftUpAngle = server.arg("liftUp").toInt();
	if (server.hasArg("stepDelay")) stepDelayMs = server.arg("stepDelay").toInt();
	if (server.hasArg("restDelay")) restDelayMs = server.arg("restDelay").toInt();
	if (server.hasArg("pumpSpeed")) pumpSpeed = server.arg("pumpSpeed").toInt();
	if (server.hasArg("maxFill")) maxFillingTime = server.arg("maxFill").toInt();

	for(int i = 0; i < NUM_TOTAL_POSITIONS; i++) {
		char argName[10];
		sprintf(argName, "rot%d", i);
		if (server.hasArg(argName)) {
			rotationAngles[i] = server.arg(argName).toInt();
		}
	}
	
	// Sicherheitsbegrenzung der Werte ---
	liftDownAngle = constrain(liftDownAngle, 0, 180);
	liftUpAngle = constrain(liftUpAngle, 0, 180);
	pumpSpeed = constrain(pumpSpeed, 0, 255); // PWM darf max 255 sein
	// -------------------------------------------

	saveConfiguration(); 
	
	String response = "Einstellungen gespeichert! Das Geraet startet neu, um die neuen Winkel anzuwenden. <meta http-equiv='refresh' content='5; url=/'><p>Startet in 5 Sekunden neu...</p>";
	server.send(200, "text/html", response);
	
	ESP.restart(); 
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

bool moveServoGradually(Servo &servo, int &currentAngle, int targetAngle, unsigned long currentMillis, int positionTimerIndex) {
	if (currentAngle == targetAngle) {
		return true; 
	}

	int timerIndex = (positionTimerIndex == -1) ? 0 : positionTimerIndex; 

	if (currentMillis - startTime[timerIndex] >= stepDelayMs) {
		startTime[timerIndex] = currentMillis; 

		if (currentAngle < targetAngle) {
			currentAngle += angleStep;
		} else {
			currentAngle -= angleStep;
		}

		if (abs(targetAngle - currentAngle) < angleStep) {
			currentAngle = targetAngle;
		}

		servo.write(currentAngle);
	}
	return false; 
}

bool moveServoGraduallyRest(Servo &servo, int &currentAngle, int targetAngle, unsigned long currentMillis) {
	extern unsigned long restProcessStartTime; // Greift auf den globalen Timer zu
	
	if (currentAngle == targetAngle) {
		return true; 
	}

	if (currentMillis - restProcessStartTime >= stepDelayMs) {
		restProcessStartTime = currentMillis; // Timer aktualisieren

		if (currentAngle < targetAngle) {
			currentAngle += angleStep;
		} else {
			currentAngle -= angleStep;
		}

		if (abs(targetAngle - currentAngle) < angleStep) {
			currentAngle = targetAngle;
		}

		servo.write(currentAngle);
	}
	return false; 
}

//================================================================================
// ZUSTANDSFUNKTIONEN
//================================================================================
void handleSensorLogic(int position, unsigned long currentMillis) {
	bool isGlassPresent = (digitalRead(SENSOR_PINS[position]) == LOW);
	
	// ACHTUNG: Der Reset bei Entfernen des Glases muss jetzt das gesamte System zurücksetzen, 
	// wenn das befüllte Glas entfernt wird und gerade kein anderes Glas bearbeitet wird.
	
	bool shouldReset = !isGlassPresent && (ledState[position] != LED_OFF);

	if (shouldReset) {
		if (processState[position] == PROCESS_PUMP_ON) {
			ledcWrite(PinENA, 0); 
		}
		
		ledState[position] = LED_OFF;
		processState[position] = PROCESS_IDLE;
		
		// NEU: Wenn das gerade befüllte oder zuletzt befüllte Glas entfernt wird, 
		// wird der aktive Prozess zurückgesetzt, ABER nur wenn das System NICHT gerade 
		// ein anderes Glas aktiv befüllt.
		if (currentProcessingPosition == position) {
			if (!isSystemBusy) { // Nur wenn keine Kette läuft
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
	
	// WICHTIG: Bearbeite nur das Glas, das gerade als AKTIV markiert ist.
	if (position != currentProcessingPosition) {
		return;
	}
	
	switch (processState[position]) {
		
		case PROCESS_CONFIRMED:
			if (currentMillis - startTime[position] >= 100) { 
				
				// Servos anhängen (sollte bereits in loop() passiert sein, hier als Sicherheit)
				if (!servoLift.attached()) {
					servoLift.attach(SERVO_LIFT_PIN);
				}
				if (!servoRotate.attached()) {
					servoRotate.attach(SERVO_ROTATE_PIN);
				}

				processState[position] = PROCESS_LIFT_UP;
				// Da die Leiter in PROCESS_LIFT_DOWN des vorherigen Glases bereits unten war, 
				// setzen wir den Startwinkel manuell, um eine flüssigere Bewegung zu gewährleisten.
				currentLiftAngle[position] = liftDownAngle; 
				startTime[position] = currentMillis; 
			}
			break;
			
		case PROCESS_LIFT_UP:
			if (moveServoGradually(servoLift, currentLiftAngle[position], liftUpAngle, currentMillis, position)) {
				processState[position] = PROCESS_ROTATE;
				currentRotateAngle[position] = servoRotate.read(); 
				startTime[position] = currentMillis; 
			}
			break;

		case PROCESS_ROTATE:
			if (moveServoGradually(servoRotate, currentRotateAngle[position], rotationAngles[position], currentMillis, position)) {
				processState[position] = PROCESS_LIFT_DOWN;
				startTime[position] = currentMillis; 
			}
			break;

		case PROCESS_LIFT_DOWN:
			if (moveServoGradually(servoLift, currentLiftAngle[position], liftDownAngle, currentMillis, position)) {
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

				// NEU: setze die LED sofort auf grün
				ledState[position] = LED_GREEN; 
				startTime[position] = currentMillis; 
			}
			break;
			
		case PROCESS_PUMP_OFF:
			if (currentMillis - startTime[position] >= FINISH_TIME) {
				
				// --- NEUE LOGIK FÜR SEQUENZIELLES FÜLLEN ---
				
				// 1. Finde das nächste bereitstehende Glas (PROCESS_CONFIRMED)
				int nextGlass = -1;
				for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
					// Suche nach dem nächsten Glas, das bereit ist und noch nicht befüllt wurde
					if (processState[i] == PROCESS_CONFIRMED) { 
						nextGlass = i;
						break;
					}
				}
				
				// 2. Setze den Zielwinkel für die MECHANIK
				if (nextGlass != -1) {
					// Nächstes Glas gefunden: Ziel ist dessen Winkel
					currentMechanismTargetAngle = rotationAngles[nextGlass];
					// WICHTIG: currentProcessingPosition wechselt zum nächsten Glas
					currentProcessingPosition = nextGlass;
				} else {
					// KEIN GLAS mehr gefunden: Ziel ist die Ruheposition
					currentMechanismTargetAngle = rotationAngles[REST_POSITION_INDEX];
				}

				// 3. Starte die Bewegung (Leiter hoch) für das GLAS, das gerade gefüllt wurde (position)
				processState[position] = PROCESS_RETURN_LIFT_UP; 
				currentLiftAngle[position] = servoLift.read(); 
				startTime[position] = currentMillis; 
			}
			break;

		// --- ÜBERARBEITETE RÜCKKEHRZUSTÄNDE (Jetzt als "MOVE TO NEXT OR REST") ---
		case PROCESS_RETURN_LIFT_UP:
			if (moveServoGradually(servoLift, currentLiftAngle[position], liftUpAngle, currentMillis, position)) {
				processState[position] = PROCESS_RETURN_ROTATE;
				currentRotateAngle[position] = servoRotate.read(); 
				startTime[position] = currentMillis; 
			}
			break;

		case PROCESS_RETURN_ROTATE:
			// Verwende den globalen Zielwinkel
			if (moveServoGradually(servoRotate, currentRotateAngle[position], currentMechanismTargetAngle, currentMillis, position)) {
				
				if (currentMechanismTargetAngle == rotationAngles[REST_POSITION_INDEX]) {
					// ZUR RUHEPOSITION ZURÜCKGEKEHRT: Endgültiger Abschluss
					processState[position] = PROCESS_RETURN_LIFT_DOWN;
				} else {
					// ZUM NÄCHSTEN GLAS GEFAHREN: Wechsel zum Füllprozess des nächsten Glases
					
					// 1. Setze den Zustand des JETZT GEFÜLLTEN Glases auf 'PROCESS_COMPLETE'
					processState[position] = PROCESS_COMPLETE; 
					
					// 2. Starte den Füllprozess für das NEUE Glas (jetzt in currentProcessingPosition)
					// Initialisiere den Lift-Winkel des neuen Glases auf den aktuellen (oben)
					currentLiftAngle[currentProcessingPosition] = liftUpAngle; 
					// Setze den Zustand des neuen Glases auf den nächsten Schritt: Leiter runter zur Füllposition.
					processState[currentProcessingPosition] = PROCESS_LIFT_DOWN;
					
					// Die Logik kehrt nun zum Anfang der switch-Anweisung zurück, wo das neue Glas 
					// (da currentProcessingPosition darauf zeigt) im nächsten Loop-Durchlauf bei PROCESS_LIFT_DOWN weitermacht.
				}
				startTime[position] = currentMillis; 
			}
			break;
			
		case PROCESS_RETURN_LIFT_DOWN:
			if (moveServoGradually(servoLift, currentLiftAngle[position], liftDownAngle, currentMillis, position)) {
				
				// Nur in diesem Zustand, wenn ZUR RUHEPOSITION ZURÜCKGEKEHRT wurde.
				processState[position] = PROCESS_COMPLETE; 
				
				// System freigeben und Servos abschalten
				isSystemBusy = false; 			
				currentProcessingPosition = -1; 	
				
				servoLift.detach();
				servoRotate.detach();
			}
			break;
			
		case PROCESS_COMPLETE:
			// Glas wurde befüllt und der Mechanismus ist weitergefahren ODER die Ruherückkehr ist abgeschlossen.
			break;
	}
}

void handleRestingTimeout(unsigned long currentMillis) {
	// NEU: Der Ruhe-Timeout soll nicht ablaufen, wenn eine Füll-Sequenz läuft.
	if (isSystemBusy) {
		restState = REST_IDLE;
		return;
	}
	
	if (restState == REST_IDLE) {
		// Hier verwenden wir jetzt die dedizierten Variablen zur Positionsprüfung
		bool isLiftRest = (restModeCurrentLiftAngle == liftDownAngle);
		bool isRotateRest = (restModeCurrentRotateAngle == rotationAngles[REST_POSITION_INDEX]);

		if (!isLiftRest || !isRotateRest) {
			if (currentMillis - lastActivityTime >= restDelayMs) { 
				
				// Servos wieder anhängen
				if (!servoLift.attached()) {
					servoLift.attach(SERVO_LIFT_PIN);
				}
				if (!servoRotate.attached()) {
					servoRotate.attach(SERVO_ROTATE_PIN);
				}
				
				restState = REST_LIFT_UP; 
				restProcessStartTime = currentMillis; // Dedizierter Timerstart
			}
		}
	}

	switch (restState) {
		
		case REST_LIFT_UP:
			// Verwende dedizierte Funktion und Variablen
			if (moveServoGraduallyRest(servoLift, restModeCurrentLiftAngle, liftUpAngle, currentMillis)) {
				restState = REST_ROTATE;
				restModeCurrentRotateAngle = servoRotate.read(); // Aktuellen Winkel ablesen
				restProcessStartTime = currentMillis; 
			}
			break;

		case REST_ROTATE:
			// Verwende dedizierte Funktion und Variablen
			if (moveServoGraduallyRest(servoRotate, restModeCurrentRotateAngle, rotationAngles[REST_POSITION_INDEX], currentMillis)) {
				restState = REST_LIFT_DOWN;
				restProcessStartTime = currentMillis; 
			}
			break;

		case REST_LIFT_DOWN:
			// Verwende dedizierte Funktion und Variablen
			if (moveServoGraduallyRest(servoLift, restModeCurrentLiftAngle, liftDownAngle, currentMillis)) {
				restState = REST_IDLE; 
				
				// Servos abschalten
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
	WiFi.softAP(ap_ssid, ap_password, 6, 0, 1); // Kanal 6, nicht versteckt, max 1 Nutzer  Serial.print("Access Point gestartet. IP: ");
	Serial.println(WiFi.softAPIP());

	// 3. Webserver initialisieren
	server.on("/", handleRoot);
	server.on("/save", handleSave);
	server.begin();

	// 4. Hardware Initialisierung

	// Initialisierung der Eingänge (Sensoren und Poti)
	for (int i = 0; i < NUM_GLAS_POSITIONS; i++) {
		pinMode(SENSOR_PINS[i], INPUT_PULLUP);
	}
	pinMode(POTI_PIN, INPUT); 
	
	// Pumpe (L298n-Modul) Setup - Hardware PWM für die Geschwindigkeit (ENA)
	ledcAttach(PinENA, 5000, 8); 
	
	pinMode(PinIN1, OUTPUT);
	pinMode(PinIN2, OUTPUT);
	digitalWrite(PinIN1, LOW); 
	digitalWrite(PinIN2, LOW);

	// NeoPixel initialisieren
	strip.begin();
	strip.setBrightness(60);
	strip.show();

	// Servos initialisieren
	servoLift.attach(SERVO_LIFT_PIN);
	servoRotate.attach(SERVO_ROTATE_PIN);
	
	// 5. KRITISCHE SICHERHEITSSEQUENZ BEIM START (BLOCKIEREND!)
	servoLift.write(liftUpAngle); 
	delay(MOVEMENT_TIME); 
	servoRotate.write(rotationAngles[REST_POSITION_INDEX]);
	delay(MOVEMENT_TIME); 
	servoLift.write(liftDownAngle);
	delay(MOVEMENT_TIME);
	
	// Servos initial trennen, da sie in der Ruheposition sind
	servoLift.detach();
	servoRotate.detach();
	
	// Initialisiere die globalen Winkel-Variablen mit der aktuellen Ruheposition
	for(int i=0; i < NUM_GLAS_POSITIONS; i++) {
		currentLiftAngle[i] = liftDownAngle;
		currentRotateAngle[i] = rotationAngles[REST_POSITION_INDEX];
		processState[i] = PROCESS_IDLE; // Sicherstellen, dass alle States auf IDLE stehen
	}
	// Initialisiere die dedizierten Ruhezustands-Variablen
	restModeCurrentLiftAngle = liftDownAngle;
	restModeCurrentRotateAngle = rotationAngles[REST_POSITION_INDEX];

	lastActivityTime = millis(); 
}

//================================================================================
// LOOP
//================================================================================
void loop() {
	unsigned long currentMillis = millis();

	// WICHTIG: Webserver muss ständig auf neue Anfragen prüfen
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
				
				// Servos anhängen, falls sie im Ruhemodus getrennt waren
				if (!servoLift.attached()) {
					servoLift.attach(SERVO_LIFT_PIN);
				}
				if (!servoRotate.attached()) {
					servoRotate.attach(SERVO_ROTATE_PIN);
				}
				
				break; 				
			}
		}
	}

	// 5. Nur den EINEN AKTIVEN Prozess abarbeiten
	if (currentProcessingPosition != -1) {
		// Die Logik steuert nun sequenziell von Glas zu Glas, 
		// indem sie currentProcessingPosition intern in handleFillingProcess wechselt.
		handleFillingProcess(currentProcessingPosition, currentMillis, fillingDuration);
	}

	// 6. Automatische Ruherückkehr bei Inaktivität
	handleRestingTimeout(currentMillis);

	// 7. LEDs aktualisieren
	updateNeoPixels(currentMillis);
}