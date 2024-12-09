#include "Servo.h"                                                                                        // Servo Bibliothek einbinden
#include <Adafruit_NeoPixel.h>                                                                            // 1000uF Kondensator zwischen + und - (5V), 300-500 OHM Widerstand in der Datenleitung!

Servo ServoDrehkranz;                                                                                     // Servo-Objekte anlegen
Servo ServoLeiter;                                                                                        // Servo-Objekte anlegen

//Pins
const byte PinKontaktGlas[] = {2, 3, 4, 5, 6, 7};                                                         // Pins der IR-Kontakte
const byte PinServoDrehkranz = 47;                                                                        // Servo-Pin Drehkranz
const byte PinServoLeiter = 45;                                                                           // Servo-Pin Leiter
const byte PinLed = 51 ;                                                                                  // LED Pin (Neopixel)
const int PinENA = A10;                                                                                   // EnA-Pin Motortreiber (PWM-Ansteuerung)
const byte PinIN1 = 29;                                                                                   // Pin IN1 Motortreiber
const byte PinIN2 = 31;                                                                                   // Pin IN2 Motortreiber
const int PinPoti = A11;                                                                                  // Pin für Poti zur Füllmenge


// Servovariablen für Drehkranz und Leiter
int ServoDrehkranzRuhestellung;
int ServoMicrosDrehkranz = ServoDrehkranzRuhestellung = 990;                                              // Istwinkel (in Micros) bei Start und in Ruhestellung
int ServoWinkelLeiter;
int ServoWinkelLeiterRuhestellung = ServoWinkelLeiter = 3;                                                // Winkel in Ruhestellung und IstWinkel (in Grad) bei Start
byte ServoWinkelLeiterHoch = 50;                                                                          // Leiter angehoben (In Grad)
byte ServoGeschwingigkeitDrehkranz = 2;                                                                   // Höhere Werte verlangsamen die Servos
byte ServoGeschwingigkeitLeiter = 30;                                                                     // Höhere Werte verlangsamen die Servos

// Winkel und Winkelberechnungen
int WinkelGlas;
int WinkelErstesGlas = WinkelGlas = 1270;                                                                 // Position des ersten Glases angeben - Im weiteren Programmablauf gibt diese Variable an, welches Glas (Winkel in Micros) angefahren wird (Funktion Tanken)
int WinkelLetztesGlas = 2270;                                                                             // Position des letzten Glases
int SchwenkWinkelProGlas = (WinkelLetztesGlas - WinkelErstesGlas) / (sizeof(PinKontaktGlas) - 1);         // Berechnung für einen gleichmäßigen Schwenkwinkel

// Pumpe
int PumpeIstAn = 0;                                                                                       // Merker
unsigned long PumpenTimer;                                                                                // Merker für Timer der Pumpe
int PumpenGeschwindigkeit = 175;                                                                          // Mit Werten von 130 bis 255 läuft die Pumpe bei den Tests 
int PumpenStandardZeit = 6000;                                                                            // Standard laufzeit der Pumpe (kann durch Poti variiert werden)
int PotiWert;                                                                                             // Variable für den Wert des Potis. Zur Regulierung der Pumpzeit.

// Blaulicht
const int Blaulicht[] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, -1};                          // Blitzabfolge
int ON = 0;                                                                                               // Merker fürs Blaulicht
int Ein = 0;                                                                                              // Merker fürs Blaulicht
int Zaehler = 0;                                                                                          // Wird in der Funktion Blitzer benötigt. Dias Array Blaulicht wird damit Stück für Stück abgearbeitet
int LeerlaufZeit = 1000;                                                                                  // Nach dieser Ruhezeit, in millis(), fährt die Drehleiter in die Ruhestellung zurück
unsigned long Merker = 0;                                                                                 // Merker für die Verzögerung beim Blaulicht (Tempo) - Dieser Variable wird der Wert von millis() übergeben. 
int BlaulichtAnforderung = 0;                                                                             // 0 = Kein Blaulicht, 1 = Angefordert, 2 = Baulicht durchläuft die Schleife
unsigned long Leerlauftimer = 0;                                                                          // Merker für die Ruhezeit, nach Ablauf der Zeit fährt die Drehleiter in die Ruhestellung zurück
const unsigned long BlaulichtTempo = 36;                                                                  // Höhere Werte verlangsamen den Blitz

//Neopixel
const int AnzahlLeds = 6;                                                                                 // Anzahl LEDs
const int LedHelligkeit = 200;                                                                            // Helligkeit 0-255
Adafruit_NeoPixel LedStreifen(AnzahlLeds, PinLed, NEO_GRB + NEO_KHZ800);                                  // Objekt LedStreifen anlegen

// Sonstige Variablen
unsigned long GlasJetzt[] = {0, 0, 0, 0, 0, 0};
int GlasDefinitionen[] = {0, 0, 0, 0, 0, 0};                                                              // 6 Werte für 6 Gläser - Hier werden die Zustände definiert
int Schritt = 1;                                                                                          // Diese Variable steuert die Bewegungsabläufe der Servos (Leiter hoch(1), Drehen(2), Leiter runter(3))
int InArbeit = 0;                                                                                         // Diese Variable sorgt dafür dass erst ein Glas befüllt wird, dann das Nächste
unsigned long Timer = 0;                                                                                  // Dieser Variable wird der Wert von millis() übergeben - für die Wartezeit bevor das Glas befüllt wird
unsigned long StandzeitNeuesGlas = 2000;                                                                  // Wie lange muss das Glas dort stehen, bevor es befüllt wird? Angabe in Millisekunden.
unsigned long LeiterTimer = millis();                                                                     // Merker - Zuständig für die Geschwindigkeit der Servos
unsigned long DrehkranzTimer = millis();                                                                  // Merker - Zuständig für die Geschwindigkeit der Servos
boolean DEBUG = true;                                                                                     // Für die Seriellen ausgaben, zur Fehlersuche
int TEXT = 0;                                                                                             // Variable zur vermeidung 1000-Facher ausgaben in der Debug-Konsole

//--------------------------------------------------------------------------------SETUP--------------------------------------------------------------------------------
void setup() {
  
  ServoDrehkranz.attach(PinServoDrehkranz);                                                               // ServoDrehkranz Pin zuweisen
  ServoLeiter.attach(PinServoLeiter);                                                                     // ServoLeiter Pin zuweisen

  pinMode(PinKontaktGlas[0], INPUT_PULLUP);                                                               // Pin für IR-Kontakt definieren, internen Pullup-Widerstand aktivieren
  pinMode(PinKontaktGlas[1], INPUT_PULLUP);                                                               // Pin für IR-Kontakt definieren, internen Pullup-Widerstand aktivieren
  pinMode(PinKontaktGlas[2], INPUT_PULLUP);                                                               // Pin für IR-Kontakt definieren, internen Pullup-Widerstand aktivieren
  pinMode(PinKontaktGlas[3], INPUT_PULLUP);                                                               // Pin für IR-Kontakt definieren, internen Pullup-Widerstand aktivieren
  pinMode(PinKontaktGlas[4], INPUT_PULLUP);                                                               // Pin für IR-Kontakt definieren, internen Pullup-Widerstand aktivieren
  pinMode(PinKontaktGlas[5], INPUT_PULLUP);                                                               // Pin für IR-Kontakt definieren, internen Pullup-Widerstand aktivieren
  pinMode(PinENA, OUTPUT);                                                                                // Pin für Motorsteuergetät definieren
  pinMode(PinIN1, OUTPUT);                                                                                // Pin für Motorsteuergetät definieren
  pinMode(PinIN2, OUTPUT);                                                                                // Pin für Motorsteuergetät definieren

  digitalWrite(PinIN1, LOW);                                                                              // Motorsteuergerät vorbereiten für motor-rechtslauf
  digitalWrite(PinIN2, LOW);                                                                              // Motorsteuergerät vorbereiten für motor-rechtslauf
  analogWrite(PinENA, PumpenGeschwindigkeit);                                                             // Pumpengeschwindigkeit setzen

  
  Serial.begin(115200);                                                                                   // Wird nur für die serielle Ausgabe benötigt (z.B. beim debuggen)
  Serial.println("FireFiller - 04.12.2024");                                                              // Datum der Version

  LedStreifen.begin();                                                                                    // Instanz starten
  LedStreifen.setBrightness(LedHelligkeit);                                                               // Helligkeit setzen
  LedStreifen.show();                                                                                     // Daten an Streifen übergeben (in diesem Fall: Streifen ausschalten)
  
  ServoLeiter.write(ServoWinkelLeiterRuhestellung);                                                       // Servo in Ruhestellung bringen
  ServoDrehkranz.writeMicroseconds(ServoDrehkranzRuhestellung);                                           // Servo in Ruhestellunng bringen

}

//--------------------------------------------------------------------------------LOOP--------------------------------------------------------------------------------
void loop() {
  Check();
  Tanken();
  Leerlaufcheck();
}

//--------------------------------------------------------------------------------FUNKTION CHECK--------------------------------------------------------------------------------
void Check(){                                                                                             // Livezustände checken und mit gemerkten Zuständen vergleichen. Stati und LEDs nach Situation verändern. 
  
  for (int i = 0; i < sizeof(PinKontaktGlas); i++) {                                                      // Kontaktnummer zum Auslesen festlegen
    if (!digitalRead(PinKontaktGlas[i])) {                                                                // 0 = Glas erkannt, 1 = Kein Glas
      switch (GlasDefinitionen[i]) {                                                                      // Mit Switch bestimmen
        case 0:                                                                                           // 0 = Vormals wurde kein Glas erkannt, nun steht hier eins! --> Licht rot, Timer starten, Status auf 1 setzen.
          if(!GlasJetzt[i]) {
            GlasJetzt[i] = millis();
            Serial.println("MILLIS WURDEN GESETZT!");
          } else if(millis() >= GlasJetzt[i] + 500) {
            Leerlauftimer = millis();
            LedStreifen.setPixelColor(i, LedStreifen.Color(255, 0, 0));                                   // Streifen rot
            LedStreifen.show();                                                                           // Pixel schalten
            Timer = millis();                                                                             // Timer Starten
            GlasDefinitionen[i] = 1;                                                                      // Neuer Status wird gesetzt

            if (DEBUG) {
              Serial.print("Glas "); Serial.print(i+1); Serial.print(" wurde akzeptiert!");Serial.println("Zustand wurde auf 1 gesetzt.");
              Serial.println("LED-Streifen wurde auf ROT gesetzt.");
              Serial.println("Timer läuft (Standzeit = 2 sekunden).");
            }

          }
          break;

        case 1:                                                                                           // 1 = Glas steht noch. Timer abwarten -->dann Status auf 2 setzen
          Leerlauftimer = millis();
          if (Timer + StandzeitNeuesGlas < millis()) {
            GlasDefinitionen[i] = 2;
            if(DEBUG) {
              Serial.println("2 Sekunden sind abgelaufen und das Glas steht noch! Zustand auf 2 gesetzt.");
            }
          }
          break;

        case 2:                                                                                           // 2 = Blaulicht an. Funktion Tanken steuert die Servos und dann wird der Status auf 3
          Leerlauftimer = millis();                                                                              
          if (BlaulichtAnforderung == 0) {
            BlaulichtAnforderung = 1;
            if(DEBUG) {
              Serial.println("Blaulicht wurde eingeschaltet.");
            }
          }
          break;

        case 3:                                                                                           // 3 = Timer starten, Status 4 setzen
          PumpenTimer = millis();
          Leerlauftimer = millis();
          GlasDefinitionen[i] = 4;
          if(DEBUG) {
            Serial.println("Pumpentimer wurde gesetzt. Zustand auf 4.");
          }
          break;

        case 4:
          Pumpen(i, PumpenStandardZeit);
          Leerlauftimer = millis();
          break;

        case 5:                                                                                           // 5 = Glas ist voll! Blaulicht abschalten und Licht grün. Status auf 4 setzen.
          BlaulichtAnforderung = 0;
          LedStreifen.setPixelColor(i, LedStreifen.Color(0, 255, 0));
          LedStreifen.show();
          GlasDefinitionen[i] = 6;
          if(DEBUG) {
            Serial.println("Licht wurde grün geschaltet. Zustand auf 6.");
          }
          break;
        
        case 6:                                                                                           // 6 = Glas ist voll, licht ist bereits grün... Nichts machen!
          break;
      }
    } else if (digitalRead(PinKontaktGlas[i])) {                                                          // Kein Glas erkannt, vorher stand hier jedoch eins...
      if(GlasDefinitionen[i] > 0) {
        Leerlauftimer = millis();
        GlasDefinitionen[i] = 0;
        GlasJetzt[i] = 0;                                                                                 // Status auf 0 setzen
        LedStreifen.setPixelColor(i, LedStreifen.Color(0, 0, 0));                                         // LEDs definieren
        LedStreifen.show();                                                                               // und schalten (aus)
        BlaulichtAnforderung = 0;                                                                         // Variable zurücksetzen
        Schritt = 1;                                                                                      // Schritt wieder auf 1 setzen (Leiter hoch)
        digitalWrite(PinIN1, LOW);                                                                        // Pumpe aus
        PumpeIstAn = 0;
        if(DEBUG) {
          Serial.println("Glas wurde nicht mehr erkannt! --> Zustand = 0. Licht aus. Blaulicht auch. Pumpe aus!");
        }
      }else {
        GlasJetzt[i] = 0;
      }
    }
  }
  Blitzer();
}



//--------------------------------------------------------------------------------FUNKTION TANKEN--------------------------------------------------------------------------------
void Tanken() {
  int x = ((WinkelGlas - WinkelErstesGlas) / SchwenkWinkelProGlas);                                       // Der Winkel des Leiterdrehkranzes gibt an welches Glas betankt werden soll (0-5)
  if (x < (sizeof(PinKontaktGlas))) {                                                                     // Hier wird kontrolliert ob es diesen Wert in derm Array noch gibt.
    int LiveZustand = digitalRead(PinKontaktGlas[x]);                                                     // Kontakt auslesen und Wert in Variable schreiben
    if (LiveZustand == 0 && GlasDefinitionen[x] == 2 && (InArbeit == 0 || InArbeit == x)) {               // Wenn ein Glas erkannt wird UND das Glas befüllt werden soll (Status 2) UND die Variable InArbeit "0" ODER "die Nummer des Feldes" hat DANN weiter
      if (InArbeit == 0) {                                                                                // Die Servos sollen sich bewegen. Wenn InArbeit = 0 dann
        InArbeit = x;                                                                                     // muss InArbeit die nummer des Glases erhalten um die Bewegungen für dieses Glas komplett abzuarbeiten
      }
      switch (Schritt) {                                                                                  //Schritt 1 = Leiter hoch, 2 = Drehen, 3 = Leiter runter
        case 1:                                                                                           // Leiter hoch
          LeiterBewegen(ServoWinkelLeiterHoch, ServoGeschwingigkeitLeiter);
          if(DEBUG) {
            if(TEXT != 2) {
              Serial.println("Leiterbewegt sich hoch.");
              TEXT = 2;
            }
          }
          break;
        
        case 2:                                                                                           // Drehkranz drehen
          DrehkranzBewegen(WinkelGlas, ServoGeschwingigkeitDrehkranz);
          if(DEBUG) {
            if(TEXT != 3) {
              Serial.println("Drehkranz bewegt sich.");
              TEXT = 3;
            }
          }
          break;
        
        case 3:                                                                                           // Leiter runter
          LeiterBewegen(0, ServoGeschwingigkeitLeiter);
          if(DEBUG) {
            if(TEXT != 4) {
              Serial.println("Leiter bewegt sich runter.");
              TEXT = 4;
            }
          }
          break;
      }
    }
  }                          
  WinkelGlas += SchwenkWinkelProGlas;                                                                     // Schwenkwinkel um ein Glas erhöhen
    if (WinkelGlas > WinkelLetztesGlas) {                                                                 // Wenn SollWinkel größer als WinkelLetztesGlas
    WinkelGlas = WinkelErstesGlas;                                                                        // Winkel wieder auf erstes Glas zurücksetzen
  }
}

//--------------------------------------------------------------------------------FUNKTION PUMPEN--------------------------------------------------------------------------------
int Pumpen (int GlasNummer, int Pumpdauer) {
  PotiWert = analogRead(PinPoti);                                                                         // Poti auslesen
  if(DEBUG) {
    if(TEXT != PotiWert && TEXT != PotiWert +1 && TEXT != PotiWert - 1) {
      Serial.print("Poti steht auf "); Serial.println(PotiWert);
      TEXT = PotiWert;
    }
  }
  if (PumpenTimer < millis() && PumpeIstAn == 0){
    digitalWrite(PinIN1, HIGH);                                                                           // Pumpe einschalten
    PumpeIstAn = 1;
    if(DEBUG) {
      Serial.println("Pumpe ist an!");
    }
  }
  if (PumpenTimer + Pumpdauer + (PotiWert*4) < millis() && PumpeIstAn == 1) {                             // Nach abgelaufener Zeit (incl. PotiWert)
    digitalWrite(PinIN1, LOW);                                                                            // Pumpe aus
    PumpeIstAn = 0;
    GlasDefinitionen[GlasNummer] = 5;                                                                     // Neuen Status setzen
    if(DEBUG) {
      Serial.print("Pumpvorgang abgeschlossen, Pumpe aus! Zustand auf 5.");
    }
  }
}

//--------------------------------------------------------------------------------FUNKTION BLITZER--------------------------------------------------------------------------------
void Blitzer() {
  if (BlaulichtAnforderung == 1){                                                                         // Wenn Blaulichtanforderung = 1 ist, soll der Blitzer laufen und die Servos angesteuert werden.
    Merker = millis();                                                                                    // Merker setzen
    Zaehler = 0;                                                                                          // Variable zum Auslesen der Blitzabfolge
    BlaulichtAnforderung = 2;                                                                             // Blaulichtanforderung auf 2 setzen, sonst startet der Timer bei jedem durchgang wieder von vorne.
  }
  if(BlaulichtAnforderung == 2  && millis() >= (Merker + BlaulichtTempo)) {                               // Erst ausführen wenn der Timer abgelaufen ist
    for (int h = 0; h < sizeof(PinKontaktGlas); h++) {
      if (GlasDefinitionen[h] > 1 && GlasDefinitionen[h] < 5 && Blaulicht[Zaehler] == 1 && Ein == 0) {
        LedStreifen.setPixelColor(h, LedStreifen.Color(0, 0, 255));
        ON = 1;
      } 
      if (GlasDefinitionen[h] > 1 && GlasDefinitionen[h] < 5 && Blaulicht[Zaehler] == 0 && Ein == 1) {    // Wenn Blaulicht[i] = 0 dann Licht ausschalten und Merken dass es aus ist
        LedStreifen.setPixelColor(h, LedStreifen.Color(0, 0, 0));
        ON = 0;
      }
    }
    if ((Ein == 0 && ON == 1) || (Ein == 1 && ON == 0)) {                                                 // Wenn neue Befehle warten, dann 
    LedStreifen.show();                                                                                   // Licht schalten
    }
    Ein = ON;
    Merker = millis();                                                                                    // Timer zurücksetzen
    Zaehler++;                                                                                            // Zähler erhöhen
    if (Zaehler > sizeof(Blaulicht) / 2) {                                                                // Wenn Zähler zu groß wird
      Zaehler = 0;                                                                                        // Zähler zurücksetzen
    }
  }
}

//--------------------------------------------------------------------------------FUNKTION LEERLAUFCHECK--------------------------------------------------------------------------------
void Leerlaufcheck() {
  if (Leerlauftimer + LeerlaufZeit < millis() && (ServoMicrosDrehkranz > ServoDrehkranzRuhestellung || ServoWinkelLeiter > ServoWinkelLeiterRuhestellung)) {
    if(DEBUG) {
      if(TEXT != 5) {
        Serial.println("Leiter fährt in Ruheposition!");
        TEXT = 5;
      }
    }
    switch (Schritt) {
      case 1:
        LeiterBewegen(ServoWinkelLeiterHoch, ServoGeschwingigkeitLeiter);
        break;
        
      case 2:
        DrehkranzBewegen(ServoDrehkranzRuhestellung, ServoGeschwingigkeitDrehkranz);
        break;
        
      case 3:
        LeiterBewegen(ServoWinkelLeiterRuhestellung, ServoGeschwingigkeitLeiter);
        break;
    }
  }
}

//--------------------------------------------------------------------------------FUNKTION LEITERBEWEGEN--------------------------------------------------------------------------------
int LeiterBewegen (int Soll, unsigned long Verzoegerung) {                                                // Funktion zum bewegen der Leiter
  if (ServoWinkelLeiter < Soll && LeiterTimer + Verzoegerung < millis() && !PumpeIstAn) {                 // Wenn der SollWinkel nicht erreicht ist und die Wartezeit abgelaufen ist, dann weiterdrehen
    ServoWinkelLeiter++;                                                                                  // IstWinkel um 1 erhöhen
    ServoLeiter.write(ServoWinkelLeiter);                                                                 // auf Wert drehen
    LeiterTimer = millis();                                                                               // Timer für Wartezeit neu starten
  }
  if (ServoWinkelLeiter > Soll && LeiterTimer + Verzoegerung < millis()) {                                // Wenn der SollWinkel nicht erreicht ist und die Wartezeit abgelaufen ist, dann weiterdrehen
    ServoWinkelLeiter--;                                                                                  // IstWinkel um 1 verringern
    ServoLeiter.write(ServoWinkelLeiter);                                                                 // auf Wert drehen
    LeiterTimer = millis();                                                                               // Timer für WarteZeit neu starten
  }
  if (ServoWinkelLeiter == Soll) {                                                                        // Wenn Sollwinkel erreicht ist dann 
    Schritt++;                                                                                            // zum nächsten Schritt gehen
    if (Schritt == 4) {                                                                                   // Wenn Schritt 4 oder größer (Es gibt nur 3 Schritte)
      InArbeit = 0;
      Schritt = 1;
     if (ServoMicrosDrehkranz != ServoDrehkranzRuhestellung) {
        GlasDefinitionen[((WinkelGlas - WinkelErstesGlas) / SchwenkWinkelProGlas)] = 3;                   // Da der ablauf für dieses Glas abgeschlossen ist, wird der Status neu gesetzt
      }    
    }
  }
}

//--------------------------------------------------------------------------------FUNKTION DREHKRANZ BEWEGEN--------------------------------------------------------------------------------
int DrehkranzBewegen (int Soll, int Verzoegerung) {                                                       // Funktion zum bewegen des Drehkranzes
    if (ServoMicrosDrehkranz < Soll && DrehkranzTimer + Verzoegerung < millis()) {                        // Wenn der SollWinkel nicht erreicht ist und die Wartezeit abgelaufen ist, dann weiterdrehen
    ServoMicrosDrehkranz++;                                                                               // IstWinkel um 1 erhöhen
    ServoDrehkranz.writeMicroseconds(ServoMicrosDrehkranz);                                               // auf Wert drehen
    DrehkranzTimer = millis();                                                                            // Timer für Wartezeit neu Starten
  }
    if (ServoMicrosDrehkranz > Soll && DrehkranzTimer + Verzoegerung < millis()) {                        // Wenn der SollWinkel nicht erreicht ist und die Wartezeit abgelaufen ist, dann weiterdrehen
    ServoMicrosDrehkranz--;                                                                               // IstWinkel um 1 verringern
    ServoDrehkranz.writeMicroseconds(ServoMicrosDrehkranz);                                               // auf Wert drehen
    DrehkranzTimer = millis();                                                                            // Timer für Wartezeit neu Starten
  }
    if (ServoMicrosDrehkranz == Soll) {                                                                   // Wenn der Winkel erreicht ist,
    Schritt++;                                                                                            // zum nächsten Schritt gehen
  }
}