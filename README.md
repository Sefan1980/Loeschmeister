# 🚒 Loeschmeister-Arduino MEGA 2560 – Der automatisierte Getränke-Ausschenker

Ein auf einem Arduino MEGA 2560 basierendes Projekt, das einen Getränke-Ausschenker in Form eines Feuerwehr-Drehleiterwagens steuert. Der Code implementiert eine Multitasking-Logik, die Servo-Bewegungen (Leiter rauf/runter, Drehkranz) und eine Pumpe steuert, basierend auf IR-Näherungssensoren.

---

## 🛠️ Hardware-Anforderungen (BOM)

| Komponente | Beschreibung |
| :--- | :--- |
| **Microcontroller** | Arduino Mega 2560 (oder ähnlich) |
| **Servos** | 2x Standard Servo (SG90 oder MG996R, abhängig von der Baugröße) |
| **Motortreiber** | L298N Modul (für die Pumpe) |
| **Pumpe, Lebensmittelecht** | 12V DC Wasserpumpe (oder 5V, je nach Bauweise) |
| **Lichter** | Adafruit NeoPixel oder kompatibler WS2812B LED-Streifen (6 LEDs) |
| **Sensoren** | 6x IR-Näherungssensoren |
| **Eingabe** | 1x Potentiometer (Poti) |

---

## 📌 Pin-Belegung (Pinout)

Der Sketch verwendet die folgenden GPIO-Pins des Arduino Mega:

| Funktion | Pin (GPIO) | Anmerkung |
| :--- | :--- | :--- |
| **NeoPixel Daten** | **51** | `PinLed` |
| **Servo Lift** | **45** | `PinServoLeiter` |
| **Servo Rotate** | **47** | `PinServoDrehkranz` |
| **Poti** | **A11** | Analoger Eingang |
| **Pumpe PWM** | **A10** | **`PinENA`** (Geschw.-Steuerung) |
| **Pumpe IN1** | **29** | **`PinIN1`** (Richtung) |
| **Pumpe IN2** | **31** | **`PinIN2`** (Richtung) |
| **Sensoren** | 2, 3, 4, 5, 6, 7 | (Wird im Array `PinKontaktGlas[]` definiert) |

---

## ⚙️ Software-Einrichtung (Arduino IDE)

### 1. Board-Manager
Stellen Sie sicher, dass das die Aktuellste Verion installiert ist.

### 2. Benötigte Bibliothekenie
Die folgenden Bibliotheken müssen im Bibliotheksverwalter der Arduino IDE installiert werden:

1.  **Adafruit NeoPixel**
2.  **Servo**

---

Viel Spaß mit dem Projekt!







