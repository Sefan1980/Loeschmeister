#include <Servo.h>
#include <Adafruit_NeoPixel.h>                                                                            // 1000uF Kondensator zwischen + und - (5V), 300-500 OHM Widerstand in der Datenleitung!

Servo ServoDrehkranz;                                                                                     // Servo-Objekte anlegen
Servo ServoLeiter;                                                                                        // Servo-Objekte anlegen

//Pins
const int PinKontaktGlas[] = {2, 3, 4, 5, 6, 7};                                                          // Pins der IR-Kontakte
const int PinServoDrehkranz = 47;                                                                         // Servo-Pin Drehkranz
const int PinServoLeiter = 45;                                                                            // Servo-Pin Leiter
const int PinLed = 51 ;                                                                                   // LED Pin (Neopixel)
const int PinENA = A10;
const int PinIN1 = 29;
const int PinIN2 = 31;



// Servovariablen für Drehkranz und Leiter
int ServoDrehkranzRuhestellung;
int ServoMicrosDrehkranz = ServoDrehkranzRuhestellung = 1020;                                             // Istwinkel (in Micros) bei Start und in Ruhestellung
int ServoWinkelLeiter;
int ServoWinkelLeiterRuhestellung = ServoWinkelLeiter = 2;                                                // Winkel in Ruhestellung und IstWinkel (in Grad) bei Start
int ServoWinkelLeiterHoch = 50;                                                                           // Leiter angehoben (In Grad)
int ServoGeschwingigkeitDrehkranz = 5;                                                                    // Höhere Werte verlangsamen die Servos
int ServoGeschwingigkeitLeiter = 30;                                                                      // Höhere Werte verlangsamen die Servos

// Winkel und Winkelberechnungen
int WinkelGlas;
int WinkelErstesGlas = WinkelGlas = 1300;                                                                 // Position des ersten Glases angeben - Im weiteren Programmablauf gibt diese Variable an, welches Glas (Winkel in Micros) angefahren wird (Funktion Tanken)
int WinkelLetztesGlas = 2050;                                                                             // Position des letzten Glases
int SchwenkWinkelProGlas = (WinkelLetztesGlas - WinkelErstesGlas) / ((sizeof(PinKontaktGlas)/2) - 1);     // Berechnung für einen gleichmäßigen Schwenkwinkel

// Pumpe
int PumpeIstAn = 0;
unsigned long PumpenTimer;
int PumpenGeschwindigkeit = 255;

// Blaulicht
const int Blaulicht[] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, -1};                          // Blitzabfolge
int ON = 0;                                                                                               // Merker fürs Blaulicht
int Ein = 0;                                                                                              // Merker fürs Blaulicht
int Zaehler = 0;                                                                                          // Wird in der Funktion Blitzer benötigt. Dias Array Blaulicht wird damit Stück für Stück abgearbeitet
int LeerlaufZeit = 100;                                                                                   // Nach dieser Ruhezeit, in millis(), fährt die Drehleiter in die Ruhestellung zurück
unsigned long Merker = 0;                                                                                 // Merker für die Verzögerung beim Blaulicht (Tempo) - Dieser Variable wird der Wert von millis() übergeben. 
int BlaulichtAnforderung = 2;                                                                             // 0 = Kein Blaulicht, 1 = Angefordert, 2 = Baulicht durchläuft die Schleife
unsigned long Leerlauftimer = 0;                                                                          // Merker für die Ruhezeit, nach Ablauf der Zeit fährt die Drehleiter in die Ruhestellung zurück
const unsigned long BlaulichtTempo = 36;                                                                  // Höhere Werte verlangsamen den Blitz

//Neopixel
const int AnzahlLeds = 6;                                                                                 // Anzahl LEDs
const int LedHelligkeit = 50;                                                                             // Helligkeit 0-255
Adafruit_NeoPixel LedStreifen(AnzahlLeds, PinLed, NEO_GRB + NEO_KHZ800);                                  // Objekt LedStreifen anlegen

// Sonstige Variablen
int GlasDefinitionen[] = {0,0,0,0,0,0};                                                                   // 6 Werte für 6 Gläser - Hier werden die Zustände definiert
int Schritt = 1;                                                                                          // Diese Variable steuert die Bewegungsabläufe der Servos (Leiter hoch(1), Drehen(2), Leiter runter(3))
int InArbeit = 0;                                                                                         // Diese Variable sorgt dafür dass erst ein Glas befüllt wird, dann das Nächste
unsigned long Timer = 0;                                                                                  // Dieser Variable wird der Wert von millis() übergeben - für die Wartezeit bevor das Glas befüllt wird
unsigned long StandzeitNeuesGlas = 1000;                                                                  // Wie lange muss das Glas dort stehen, bevor es befüllt wird? Angabe in Millisekunden.
unsigned long LeiterTimer = millis();                                                                     // Merker - Zuständig für die Geschwindigkeit der Servos
unsigned long DrehkranzTimer = millis();                                                                  // Merker - Zuständig für die Geschwindigkeit der Servos

//--------------------------------------------------------------------------------SETUP--------------------------------------------------------------------------------
void setup() {
  
  ServoDrehkranz.attach(PinServoDrehkranz);                                                               // ServoDrehkranz Pin zuweisen
  ServoLeiter.attach(PinServoLeiter);                                                                     // ServoLeiter Pin zuweisen

  pinMode(PinKontaktGlas[0], INPUT_PULLUP);                                                               // Pins definieren, interne Pullup-Widerstände aktivieren
  pinMode(PinKontaktGlas[1], INPUT_PULLUP); 
  pinMode(PinKontaktGlas[2], INPUT_PULLUP); 
  pinMode(PinKontaktGlas[3], INPUT_PULLUP); 
  pinMode(PinKontaktGlas[4], INPUT_PULLUP); 
  pinMode(PinKontaktGlas[5], INPUT_PULLUP); 
  pinMode(PinENA, OUTPUT);
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);

  digitalWrite(PinIN1, LOW);
  digitalWrite(PinIN2, LOW);
  analogWrite(PinENA, PumpenGeschwindigkeit);

  
Serial.begin(9600);                                                                                   // Wird nur für die serielle Ausgabe benötigt (z.B. beim debuggen)
Serial.println("Start");

  LedStreifen.begin();                                                                                    // Instanz starten
  LedStreifen.setBrightness(LedHelligkeit);                                                               // Helligkeit setzen
  LedStreifen.show();                                                                                     // Daten an Streifen übergeben (in diesem Fall: Streifen ausschalten)
  
  WinkelLetztesGlas = WinkelErstesGlas + (5 * SchwenkWinkelProGlas);                                      // Falls die Position des letzten GlasWinkels nicht 100%ig ist, wird hier korrigiert

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
  for (int i = 0; i < sizeof(PinKontaktGlas) / 2 ; i++) {                                                 // Kontaktnummer zum Auslesen festlegen
    int LiveZustand = digitalRead(PinKontaktGlas[i]);                                                     // Kontakt auslesen und Wert in Variable schreiben
    if (LiveZustand == 0) {                                                                               // 0 = Glas erkannt, 1 = Kein Glas
      switch (GlasDefinitionen[i]) {                                                                      // Mit Switch bestimmen
        case 0:                                                                                           // 0 = Vormals wurde kein Glas erkannt, nun steht hier eins! --> Licht rot, Timer starten, Status auf 1 setzen.
          LedStreifen.setPixelColor(i, LedStreifen.Color(255, 0, 0));                                     // Streifen rot
          LedStreifen.show();                                                                             // Pixel schalten
          Timer = millis();                                                                               // Timer Starten
          GlasDefinitionen[i] = 1;                                                                        // Neuer Status wird gesetzt
          break;

        case 1:                                                                                           // 1 = Glas steht noch. Timer abwarten -->dann Status auf 2 setzen
          Leerlauftimer = millis();
          if (Timer + StandzeitNeuesGlas < millis()) {
            GlasDefinitionen[i] = 2;
          }
          break;

        case 2:                                                                                           // 2 = Blaulicht an. Funktion Tanken steuert die Servos und dann wird der Status auf 3
          Leerlauftimer = millis();                                                                              
          if (BlaulichtAnforderung == 0) {
            BlaulichtAnforderung = 1;
            Serial.print("Case2");
          }
          break;

        case 3:           //Pumpe EIN, Timer starten, Status 4 setzen (Von Status 2 auf 5 wird zur Zeit mit der Funktion LeiderBewegen umgestellt!)
          PumpenTimer = millis();
          GlasDefinitionen[i] = 4;
          break;

        case 4:           //Pumpe ein- und ausschalten, Status auf 5 setzen (in Funktion Pumpen)
          Pumpen(i, 2000);
          break;

        case 5:                                                                                           // 5 = Glas ist voll! Blaulicht abschalten und Licht grün. Status auf 4 setzen.
          BlaulichtAnforderung = 0;
          LedStreifen.setPixelColor(i, LedStreifen.Color(0, 255, 0));
          LedStreifen.show();
          GlasDefinitionen[i] = 6;
          break;
        
        case 6:                                                                                           // 6 = Glas ist voll, licht ist bereits grün... Nichts machen!
          break;
      }
    } else if (LiveZustand == 1 && GlasDefinitionen[i] > 0) {                                             // Kein Glas erkannt, vorher stand hier jedoch eins...
      GlasDefinitionen[i] = 0;                                                                            // Status auf 0 setzen
      LedStreifen.setPixelColor(i, LedStreifen.Color(0, 0, 0));                                           // LEDs definieren
      LedStreifen.show();                                                                                 // und schalten (aus)
      BlaulichtAnforderung = 0;                                                                           // Variable zurücksetzen
      Schritt = 1;                                                                                        // Schritt wieder auf 1 setzen (Leiter hoch)
    }
//    Blitzer();                                                                                           // Blaulichtblitz aufrufen
  }
  Blitzer();
}

//--------------------------------------------------------------------------------FUNKTION TANKEN--------------------------------------------------------------------------------
void Tanken() {
  int x = ((WinkelGlas - WinkelErstesGlas) / SchwenkWinkelProGlas);                                       // Der Winkel des Leiterdrehkranzes gibt an welches Glas betankt werden soll (0-5)
  if (x < (sizeof(PinKontaktGlas) / 2)) {                                                                 // Hier wird kontrolliert ob es diesen Wert in derm Array noch gibt.
    int LiveZustand = digitalRead(PinKontaktGlas[x]);                                                     // Kontakt auslesen und Wert in Variable schreiben
    if (LiveZustand == 0 && GlasDefinitionen[x] == 2 && (InArbeit == 0 || InArbeit == x)) {               // Wenn Ein Glas erkannt wird UND das Glas befüllt werden soll (Status 2) UND die Variable InArbeit "0" ODER "die Nummer des Feldes" hat DANN weiter
      if (InArbeit == 0) {                                                                                // Die Servos sollen sich bewegen. Wenn InArbeit = 0 dann
        InArbeit = x;                                                                                     // muss InArbeit die nummer des Glases erhalten um die Bewegungen für dieses Glas komplett abzuarbeiten
      }
      switch (Schritt) {                                                                                  //Schritt 1 = Leiter hoch, 2 = Drehen, 3 = Leiter runter
        case 1:
          LeiterBewegen(50, ServoGeschwingigkeitLeiter);
          break;
        
        case 2:
          DrehkranzBewegen(WinkelGlas, ServoGeschwingigkeitDrehkranz);
          break;
        
        case 3:
          LeiterBewegen(0, ServoGeschwingigkeitLeiter);
          break;
      }
    }
  }                          
  WinkelGlas += SchwenkWinkelProGlas;
    if (WinkelGlas > WinkelLetztesGlas) {
    WinkelGlas = WinkelErstesGlas;    
  }
}

//--------------------------------------------------------------------------------FUNKTION PUMPEN--------------------------------------------------------------------------------
int Pumpen (int GlasNummer, int Pumpdauer) {
  if (PumpenTimer < millis() && PumpeIstAn == 0){
    Serial.println("Pumpe ist an.");
    digitalWrite(PinIN1, HIGH);
    PumpeIstAn = 1;
  }
  if (PumpenTimer + Pumpdauer < millis() && PumpeIstAn == 1) {
    Serial.println("Pumpe ist aus.");
    digitalWrite(PinIN1, LOW);
    PumpeIstAn = 0;
    GlasDefinitionen[GlasNummer] = 5;
  }
}

//--------------------------------------------------------------------------------FUNKTION BLITZER--------------------------------------------------------------------------------
void Blitzer() {
  if (BlaulichtAnforderung == 1){                                                                         // Wenn Blaulichtanforderung = 1 ist, soll der Blitzer laufen und die Servos angesteuert werden.
    Merker = millis();                                                                                    // Merker setzen
    Zaehler = 0;                                                                                          // Variable zum Auslesen der Blitzabfolge
    BlaulichtAnforderung = 2;                                                                             // Blaulichtanforderung auf 2 setzen, sonst startet der Timer bei jedem durchgang wieder von vorne.
  }
  if(BlaulichtAnforderung == 2  && millis() >= (Merker + BlaulichtTempo)) {                                // Erst ausführen wenn Der Timer abgelaufen ist
    for (int h = 0; h < sizeof(PinKontaktGlas) / 2 ; h++) {
      if (GlasDefinitionen[h] == 2 && Blaulicht[Zaehler] == 1 && Ein == 0) {
        LedStreifen.setPixelColor(h, LedStreifen.Color(0, 0, 255));
        ON = 1;
      } 
      if (GlasDefinitionen[h] == 2 && Blaulicht[Zaehler] == 0 && Ein == 1) {                              // Wenn Blaulicht[i] = 0 dann Licht ausschalten und Merken dass es aus ist
        LedStreifen.setPixelColor(h, LedStreifen.Color(0, 0, 0));
        ON = 0;
      }
    }
    if ((Ein == 0 && ON == 1) || (Ein == 1 && ON == 0)) {
    LedStreifen.show();
    }
    Ein = ON;
    Merker = millis();
    Zaehler++;
    if (Zaehler > sizeof(Blaulicht) / 2) {
      Zaehler = 0;
    }
  }
}

//--------------------------------------------------------------------------------FUNKTION LEERLAUFCHECK--------------------------------------------------------------------------------
void Leerlaufcheck() {
  if (Leerlauftimer + LeerlaufZeit < millis() && (ServoMicrosDrehkranz > ServoDrehkranzRuhestellung || ServoWinkelLeiter > ServoWinkelLeiterRuhestellung)) {
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
  if (ServoWinkelLeiter < Soll && LeiterTimer + Verzoegerung < millis() && PumpeIstAn == 0) {                                // Wenn der SollWinkel nicht erreicht ist und die Wartezeit abgelaufen ist, dann weiterdrehen
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
Serial.println("Leiter ist hoch/runter.");
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
  Serial.println("Drehkranz ist positioniert");
    Schritt++;                                                                                            // zum nächsten Schritt gehen
  }
}