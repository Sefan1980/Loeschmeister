# 🚒 Loeschmeister-ESP32 – Der automatisierte Getränke-Ausschenker

Ein auf einem ESP32 basierendes Projekt, das einen Getränke-Ausschenker in Form eines Feuerwehr-Drehleiterwagens steuert. Der Code implementiert eine Multitasking-Logik, die Servo-Bewegungen (Leiter rauf/runter, Drehkranz) und eine Pumpe steuert, basierend auf IR-Näherungssensoren.

Zudem kann die gesamte Konfiguration (Winkel, Geschwindigkeiten, Füllzeiten) über einen integrierten **Access Point (AP)** und einen **Webserver** einfach angepasst werden, ohne den Code neu kompilieren zu müssen.

---

## 🛠️ Hardware-Anforderungen (BOM)

| Komponente | Beschreibung |
| :--- | :--- |
| **Microcontroller** | ESP32 Dev Kit (oder ähnlicher) |
| **Servos** | 2x Standard Servo (SG90 oder MG996R, abhängig von der Baugröße) |
| **Motortreiber** | L298N Modul (für die Pumpe) |
| **Pumpe, Lebensmittelecht** | 12V DC Wasserpumpe (oder 5V, je nach Bauweise) |
| **Lichter** | Adafruit NeoPixel oder kompatibler WS2812B LED-Streifen (6 LEDs) |
| **Sensoren** | 6x IR-Näherungssensoren |
| **Eingabe** | 1x Potentiometer (Poti) |
| **Akku** | 1x 4S LiPo (andere Akkus sind zur Zeit nicht implementiert) |
| **Widerstände** | 1x 10kOhm, 1x 56kOhm (Spannungsteiler - bei anderen Akkus können andere Werte nötig sein!)|
| **StepDownWandler** | 2x z.B. LM2596 (Um die Spannung vom Akku auf 12V für den Motortreiber bzw. auf 5V für das Board und die Servos zu begrenzen ) |

---

## 📌 Pin-Belegung (Pinout)

Der Sketch verwendet die folgenden GPIO-Pins des ESP32:

| Funktion | Pin (GPIO) | Anmerkung |
| :--- | :--- | :--- |
| **NeoPixel Daten** | **18** | `LED_PIN` |
| **Servo Lift** | **17** | `SERVO_LIFT_PIN` |
| **Servo Rotate** | **16** | `SERVO_ROTATE_PIN` |
| **Poti** | **34** | Analoger Eingang (ADC1) |
| **Pumpe PWM** | **23** | **`PinENA`** (Geschw.-Steuerung) |
| **Pumpe IN1** | **22** | **`PinIN1`** (Richtung) |
| **Pumpe IN2** | **21** | **`PinIN2`** (Richtung) |
| **Sensoren** | 32, 33, 25, 26, 27, 14 | (Wird im Array `SENSOR_PINS` definiert) |
| **Spannungsteiler** | **35**| analogRead(AKKU_PIN) |

---

## ⚙️ Software-Einrichtung (Arduino IDE)

### 1. Board-Manager
Stellen Sie sicher, dass das **ESP32 Board Package (Version 3.0 oder neuer)** installiert ist.

### 2. Benötigte Bibliotheken
Die folgenden Bibliotheken müssen im Bibliotheksverwalter der Arduino IDE installiert werden:

1.  **Adafruit NeoPixel**
2.  **ESP32Servo** (Wichtig: Die spezielle ESP32-Version z.B. von Kevin Harrington, John K. Bennett)
3.  **WebServer** (Standard-Bibliothek für ESP32)
4.  **Preferences** (Standard-Bibliothek für ESP32 NVS)

---

## 🌐 Konfiguration per Webserver

Nach dem Hochladen des Sketches startet der ESP32 einen Access Point:

1.  Verbinden Sie Ihr Smartphone/PC mit dem WLAN: **`Loeschmeister_Konfig`**
2.  Geben Sie das Passwort ein: **`Passwort123`** (Achtung: Dies ist im Code hart codiert!)
3.  Öffnen Sie einen Browser und geben Sie die IP-Adresse ein: **`192.168.4.1`**
4.  Über die Konfigurationsseite können Sie alle Winkel, Geschwindigkeiten und Zeiten speichern, die im NVS (Nichtflüchtiger Speicher) abgelegt werden.

Viel Spaß mit dem Projekt!
