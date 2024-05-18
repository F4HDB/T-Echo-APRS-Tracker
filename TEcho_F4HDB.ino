// TEcho_F4HDB.ino
// v1.00

// Programme tracker APRS avec un module LILYGO T-ECHO.
// Adapté par Frank LOBA (www.f4hdb.fr).
// Date création : 11/02/2024.
// Dernière mise à jour : 15/04/2024.

// Programm APRS-Tracker mit einem LILYGO T-ECHO-Modul.
// Adaptiert von Frank LOBA (www.f4hdb.fr).
// Erstellungsdatum: 11.02.2024.
// Letzte Aktualisierung: 15.04.2024.

// Program APRS tracker with a LILYGO T-ECHO module.
// Adapted by Frank LOBA (www.f4hdb.fr).
// Creation date: 02/11/2024.
// Last update: 04/15/2024.


/**** INCLUSION DES BIBLIOTHEQUES GENERALES ****/
/**** EINBEZIEHUNG ALLGEMEINER BIBLIOTHEKEN ****/
/**** INCLUSION OF GENERAL LIBRARIES ****/
#include <Arduino.h>
#include <SPI.h>

#include <math.h>


/**** INCLUSION DES BIBLIOTHEQUES NECESSAIRES AU PILOTAGE DE L'ECRAN E-PAPER ****/
/**** EINSCHLUSS VON BIBLIOTHEKS, DIE ZUR STEUERUNG DES E-PAPER-BILDSCHIRMS ERFORDERLICH SIND ****/
/**** INCLUSION OF LIBRARIES REQUIRED TO CONTROL THE E-PAPER SCREEN ****/
#include <GxEPD.h>
#include <GxDEPG0150BN/GxDEPG0150BN.h>  // 1.54" b/w
#include <Fonts/FreeMonoBold9pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>


/**** INCLUSION DES BIBLIOTHEQUES NECESSAIRES AU PILOTAGE DE LA RADIO LORA ****/
/**** EINBEZIEHUNG DER ZUR STEUERUNG DES LORA-RADIOS NOTWENDIGEN BIBLIOTHEKEN ****/
/**** INCLUSION OF THE LIBRARIES NECESSARY TO CONTROL THE LORA RADIO ****/
#include <RadioLib.h>


/**** INCLUSION DES BIBLIOTHEQUES NECESSAIRES AU PILOTAGE DU GPS ****/
/**** EINBEZIEHUNG DER ZUR GPS-STEUERUNG NOTWENDIGEN BIBLIOTHEKEN ****/
/**** INCLUSION OF LIBRARIES NECESSARY FOR CONTROLLING THE GPS ****/
#include <TinyGPS++.h>


/**** DECLARATION DE LA MACRO DE CALCUL DU NUMERO DE LA BROCHE ****/
/**** ERKLÄRUNG DES PIN-NUMMER-BERECHNUNGSMAKROS ****/
/**** DECLARATION OF THE PIN NUMBER CALCULATION MACRO ****/
#ifndef _PIN_NUMBER
#define _PIN_NUMBER(_PORT, _PIN)    ((_PORT << 5) + _PIN)  // = Port * 32 + Pin.
#endif


/**** DECLARATION DES NUMEROS DES BROCHES DE L'ECRAN E-PAPER ****/
/**** ERKLÄRUNG DER E-PAPER-BILDSCHIRM-PIN-NUMMERN ****/
/**** DECLARATION OF E-PAPER SCREEN PIN NUMBERS ****/
#define E_PAPER_MISO      _PIN_NUMBER(1,6)
#define E_PAPER_MOSI      _PIN_NUMBER(0,29)
#define E_PAPER_SCLK      _PIN_NUMBER(0,31)
#define E_PAPER_CS        _PIN_NUMBER(0,30)
#define E_PAPER_DC        _PIN_NUMBER(0,28)
#define E_PAPER_RST       _PIN_NUMBER(0,2)
#define E_PAPER_BUSY      _PIN_NUMBER(0,3)
#define E_PAPER_BACKLIGHT _PIN_NUMBER(1,11)


/**** DECLARATION DES PARAMETRES RADIO LORA ****/
/**** ERKLÄRUNG DER LORA-FUNKPARAMETER ****/
/**** DECLARATION OF LORA RADIO PARAMETERS ****/
#define LORA_MISO         _PIN_NUMBER(0,23)
#define LORA_MOSI         _PIN_NUMBER(0,22)
#define LORA_SCLK         _PIN_NUMBER(0,19)
#define LORA_CS           _PIN_NUMBER(0,24)
#define LORA_RST          _PIN_NUMBER(0,25)
#define LORA_DI0          _PIN_NUMBER(0,22)
#define LORA_DIO1         _PIN_NUMBER(0,20)
#define LORA_DIO3         _PIN_NUMBER(0,21)
#define LORA_BUSY         _PIN_NUMBER(0,17)

#define LORA_FREQUENCY        433.775
#define LORA_SPREADING_FACTOR 12
#define LORA_SIGNAL_BANDWIDTH 125.0
#define LORA_CODING_RATE_4    5
#define LORA_SYNC_WORD        0x12
#define LORA_TX_POWER_DBM     20

#define LORA_START_BYTE_1 '<'
#define LORA_START_BYTE_2 0xFF
#define LORA_START_BYTE_3 0x01


/**** DECLARATION DES PARAMETRES SYSTEME ****/
/**** ERKLÄRUNG DER SYSTEMPARAMETER ****/
/**** DECLARATION OF SYSTEM PARAMETERS ****/
#define USER_TOUCH          _PIN_NUMBER(0,11)
#define USER_BUTTON         _PIN_NUMBER(1,10)
#define POWER_ENABLE        _PIN_NUMBER(0,12)

#define USER_LED_RED        _PIN_NUMBER(1,3)
#define USER_LED_GREEN      _PIN_NUMBER(1,1)
#define USER_LED_BLUE       _PIN_NUMBER(0,14)


/**** DECLARATION DES PARAMETRES GPS ****/
/**** ERKLÄRUNG DER GPS-PARAMETER ****/
/**** DECLARATION OF GPS PARAMETERS ****/
#define GPS_RX              _PIN_NUMBER(1,9)
#define GPS_TX              _PIN_NUMBER(1,8)
#define GPS_WAKEUP          _PIN_NUMBER(1,2)
#define GPS_RESET           _PIN_NUMBER(1,5)
#define GPS_PPS             _PIN_NUMBER(1,4)

#define SerialGPS           Serial2
#define GPS_BAUD_RATE       9600


/**** DECLARATION DES PARAMETRES GENERAUX ****/
/**** ERKLÄRUNG ALLGEMEINER PARAMETER ****/
/**** DECLARATION OF GENERAL PARAMETERS ****/


// Renseignez votre indicatif et SSID ici.
// Geben Sie hier Ihr Rufzeichen und Ihre SSID ein.
// Enter your callsign and SSID here.
#define APRS_CALL         "CALLSIGN-SSID"                  

#define APRS_RECIPIENT    "APLG01,WIDE1-1,WIDE2-1"
#define APRS_OVERLAY      '/'
#define APRS_SYMBOL       '['
#define APRS_COMMENT      "www.f4hdb.fr"


// Les durées sont en secondes et les distances en mètres.
// Die Dauer ist in Sekunden und die Distanz in Metern angegeben.
// The durations are in seconds and the distances in meters.
#define BACKLIGHT_DELAY     15
#define SHOW_DELAY          10
#define TOUCH_DELAY         5
#define SWAP_SCREEN_PERIOD  10

#define TRANSMIT_DELAY      60
#define TRANSMIT_DISTANCE   1000


/**** DECLARATION DES OBJETS NECESSAIRES POUR PILOTER L'ECRAN ****/
/**** ERKLÄRUNG DER ZUR STEUERUNG DES BILDSCHIRMS ERFORDERLICHEN OBJEKTE ****/
/**** DECLARATION OF OBJECTS NECESSARY TO CONTROL THE SCREEN ****/
SPIClass    DisplayPort(NRF_SPIM2, E_PAPER_MISO, E_PAPER_SCLK, E_PAPER_MOSI);
GxIO_Class  Io(DisplayPort, E_PAPER_CS, E_PAPER_DC, E_PAPER_RST);
GxEPD_Class Display(Io, E_PAPER_RST, E_PAPER_BUSY);


/**** DECLARATION DES OBJETS ET VARIABLES NECESSAIRES POUR PILOTER LA RADIO LORA ****/
/**** ERKLÄRUNG DER OBJEKTE UND VARIABLEN, DIE ZUR STEUERUNG DES LORA-RADIOS ERFORDERLICH SIND ****/
/**** DECLARATION OF OBJECTS AND VARIABLES NECESSARY TO CONTROL THE LORA RADIO ****/
SPIClass    RfPort(NRF_SPIM3, LORA_MISO, LORA_SCLK, LORA_MOSI);
SPISettings SpiSettings; 
SX1262      Radio = nullptr;

volatile bool Receive = false;
volatile bool Interrupt = true;
String        TNC2RxFrame = "";
String        TNC2TxFrame = "";
int           State;
bool          Transmit = false;


/**** DECLARATION DES OBJETS ET VARIABLES GLOBALES ****/
/**** DEKLARATION VON OBJEKTEN UND GLOBALEN VARIABLEN ****/
/**** DECLARATION OF OBJECTS AND GLOBAL VARIABLES ****/
bool          PowerEnable = false;
unsigned long BacklightTime = 0L;
unsigned long ShowTime = 0L;
unsigned long TouchTime = 0L;

TinyGPSPlus   Gps;

bool          TrackerEnable = false;
unsigned long TransmitTime = 0L;

unsigned long SwapScreenTime = 0L;
bool          ViewGps = true;

double Distance = 0.0;  
double TransmitLat = 0.0;
double TransmitLng = 0.0;


/**** DECLARATION DES FONCTIONS GLOBALES ****/
/**** ERKLÄRUNG GLOBALER FUNKTIONEN ****/
/**** DECLARATION OF GLOBAL FUNCTIONS ****/
String LatitudeAprs(RawDegrees Latitude);
String LongitudeAprs(RawDegrees Longitude);
void SetRxFlag(void);


/**** DEFINITION DE LA FONCTION DE L'INTERRUPTION POUR LA RECEPTION RADIO LORA ****/
/**** DEFINITION DER UNTERBRECHUNGSFUNKTION FÜR DEN LORA-RADIOEMPFANG ****/
/**** DEFINITION OF THE INTERRUPTION FUNCTION FOR LORA RADIO RECEPTION ****/
void SetRxFlag(void)
{
  if(Interrupt) Receive = true;
}


/**** DEFINITION DE LA FONCTION D'INITIALISATION DU MODULE LILYGO T-ECHO ****/
/**** DEFINITION DER INITIALISIERUNGSFUNKTION DES LILYGO T-ECHO-MODULS ****/
/**** DEFINITION OF THE INITIALIZATION FUNCTION OF THE LILYGO T-ECHO MODULE ****/
void setup()
{
    // Attente.
    // Warten.
    // Wait.
    delay(1500);


    // Affectation des entrées et sorties.
    // Zuordnung der Ein- und Ausgänge.
    // Assignment of inputs and outputs.
    pinMode(POWER_ENABLE, OUTPUT);
    pinMode(E_PAPER_BACKLIGHT, OUTPUT);

    pinMode(USER_BUTTON, INPUT_PULLUP);
    pinMode(USER_TOUCH, INPUT_PULLUP);

    pinMode(USER_LED_RED, OUTPUT);
    pinMode(USER_LED_GREEN, OUTPUT);
    pinMode(USER_LED_BLUE, OUTPUT);


    // Mise en marche.
    // Ein.
    // Power on.
    digitalWrite(POWER_ENABLE, HIGH);
    PowerEnable = true;


    // Initialisation des objets de l'écran.
    // Initialisierung von Bildschirmobjekten.
    // Initialization of screen objects.
    DisplayPort.begin();
    Display.init(/*115200*/);

    // Rotation de l'écran.
    // Bildschirm drehen.
    // Rotate the screen.
    Display.setRotation(3);

    // Remplissage de l'écran avec la couleur blanche (pour l'effacer).
    // Den Bildschirm mit weißer Farbe füllen (um ihn zu löschen).
    // Filling the screen with white color (to clear it).
    Display.fillScreen(GxEPD_WHITE);

    // Attribution de la couleur noir au texte affiché à l'écran.
    // Dem auf dem Bildschirm angezeigten Text die Farbe Schwarz zuweisen.
    // Assigning the color black to the text displayed on the screen.
    Display.setTextColor(GxEPD_BLACK);

    // Attribution de la police de caractères affichés à l'écran.
    // Zuweisung der auf dem Bildschirm angezeigten Schriftart.
    // Assignment of the font displayed on the screen.
    Display.setFont(&FreeMonoBold9pt7b);

    // Positionnement du curseur pour écrire à l'écran.
    // Positionierung des Cursors zum Schreiben auf dem Bildschirm.
    // Positioning the cursor to write on the screen.
    Display.setCursor(0,12);

    // Affichage du mot "Hello !" à l'écran.
    // Anzeige des Wortes „Hallo!“ auf dem Bildschirm.
    // Display of the word “Hello!” on the screen.
    Display.println(F("Hello !"));


    // Initialisation de la radio LoRa.
    // Initialisieren des LoRa-Radios.
    // Initializing the LoRa radio.
    RfPort.begin();
    Radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, RfPort, SpiSettings);
 
    if(!Radio.begin(LORA_FREQUENCY, LORA_SIGNAL_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODING_RATE_4, LORA_SYNC_WORD, LORA_TX_POWER_DBM))
    {
      // Attachement de la fonction d'interruption pour la réception radio LoRa.
      // Unterbrechungsfunktion für LoRa-Funkempfang anhängen.
      // Attach interrupt function for LoRa radio reception.
      Radio.setDio1Action(SetRxFlag);

      // Lancement de la réception radio LoRa.
      // Start des LoRa-Funkempfangs.
      // Launch of LoRa radio reception.
      if(!Radio.startReceive()) Display.println(F("LoRa OK"));
      else Display.println(F("LoRa KO"));

      // Validation de l'interruption pour la réception radio LoRa.
      // Validierung der Unterbrechung für den LoRa-Funkempfang.
      // Validation of the interruption for LoRa radio reception.
      Interrupt = true;
    }
    else Display.println(F("LoRa KO"));


    // Initialisation du GPS.
    // GPS-Initialisierung.
    // GPS initialization.
    SerialGPS.setPins(GPS_RX, GPS_TX);
    SerialGPS.begin(GPS_BAUD_RATE);
    SerialGPS.flush();

    pinMode(GPS_PPS, INPUT);
    pinMode(GPS_WAKEUP, OUTPUT);
    digitalWrite(GPS_WAKEUP, HIGH);


    // Remise à zéro du GPS.
    // GPS zurücksetzen.
    // Reset the GPS.
    delay(10);
    pinMode(GPS_RESET, OUTPUT);
    digitalWrite(GPS_RESET, HIGH);
    delay(10);
    digitalWrite(GPS_RESET, LOW);
    delay(10);
    digitalWrite(GPS_RESET, HIGH);


    // Mise à jour de l'écran pour prendre en compte les actions ci-dessus.
    // Der Bildschirm wurde aktualisiert, um die oben genannten Aktionen zu berücksichtigen.
    // Updated the screen to take into account the actions above.
    Display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);


    // Initialisation des temps.
    // Initialisierung von Zeiten.
    // Initialization of times.
    BacklightTime = millis();
    ShowTime = millis();
    TouchTime = millis();
    TransmitTime = millis();
    SwapScreenTime = millis();


    // Initialisation des états.
    // Initialisierung von Zuständen.
    // Initialization of states.
    TrackerEnable = false;
    Transmit = false;
    ViewGps = true;
}


/**** DEFINITION DE LA FONCTION D'EXECUTION DU MODULE LILYGO T-ECHO ****/
/**** DEFINITION DER AUSFÜHRUNGSFUNKTION DES LILYGO T-ECHO-MODULS ****/
/**** DEFINITION OF THE EXECUTION FUNCTION OF THE LILYGO T-ECHO MODULE ****/
void loop()
{
  // Réception d'une trame radio LoRa.
  // Empfang eines LoRa-Funkrahmens.
  // Reception of a LoRa radio frame.
  if(Receive && PowerEnable)
  {
    // Initialisation de l'indicateur de réception.
    // Initialisierung der Empfangsanzeige.
    // Initialization of the reception indicator.
    Receive = false;

    // Inhibition de l'interruption pour la réception radio LoRa.
    // Unterdrückung der Unterbrechung des LoRa-Funkempfangs.
    // Inhibition of interruption for LoRa radio reception.
    Interrupt = false;
    
    // Allumage de la LED verte.
    // Grüne LED leuchtet.
    // Green LED lights up.
    digitalWrite(USER_LED_GREEN, LOW);

    // Allumage de l'écran.
    // Bildschirmhintergrundbeleuchtung
    // Screen backlight.
    BacklightTime = millis();
    ShowTime = millis();  
    digitalWrite(E_PAPER_BACKLIGHT, HIGH);

    // Effacement de l'écran et positionnement du curseur.
    // Bildschirm löschen und Cursor positionieren. 
    // Clearing the screen and positioning the cursor.
    Display.fillScreen(GxEPD_WHITE);
    Display.setCursor(0,12);

    // Affichage de la trame, du RSSI et du SNR à l'écran.
    // Anzeige von Frame, RSSI und SNR auf dem Bildschirm.
    // Displaying frame, RSSI and SNR on screen. 
    Display.println(F("**** RX ****"));

    // Lecture de la trame.
    // Den Frame lesen.
    // Read the frame.
    State = Radio.readData(TNC2RxFrame);

    // Affichage de la trame si elle est correcte, et de ses informations à l'écran.
    // Anzeige des Frames, wenn er korrekt ist, und seiner Informationen auf dem Bildschirm.
    // Display of the frame if it is correct, and its information on the screen.
    if(State == ERR_NONE && TNC2RxFrame.charAt(0) == LORA_START_BYTE_1 && TNC2RxFrame.charAt(1) == LORA_START_BYTE_2 && TNC2RxFrame.charAt(2) == LORA_START_BYTE_3)
    {
      Display.println(TNC2RxFrame.substring(3));
  
      Display.print(F("RSSI:\t"));
      Display.print(Radio.getRSSI());
      Display.println(F(" dBm"));
  
      Display.print(F("SNR:\t"));
      Display.print(Radio.getSNR());
      Display.println(F(" dB"));
  
      Display.print(F("Length:\t"));
      Display.println(TNC2RxFrame.length());
    }

    // Gestion des erreurs de réception.
    // Management von Empfangsfehlern.
    // Management of reception errors.
    else if(State == ERR_RX_TIMEOUT) Display.println(F("TimeOut !"));

    else if(State == ERR_CRC_MISMATCH) Display.println(F("CRC Error !"));

    else
    {
      Display.print(F("Failed : "));
      Display.println(State);
    }

    // Validation de l'interruption pour la réception radio LoRa.
    // Validierung der Unterbrechung für den LoRa-Funkempfang.
    // Validation of the interruption for LoRa radio reception.
    Interrupt = true;

    // Lancement de la réception radio LoRa.
    // Start des LoRa-Funkempfangs.
    // Launch of LoRa radio reception.
    State = Radio.startReceive();

    if(State != ERR_NONE)
    {
      Display.fillScreen(GxEPD_WHITE);
      Display.setCursor(0,12);

      Display.println(F("- Start Receive -"));
      Display.print(F("Failed : "));
      Display.println(State);
    }
    
    // Mise à jour de l'écran pour prendre en compte les actions ci-dessus.
    // Der Bildschirm wurde aktualisiert, um die oben genannten Aktionen zu berücksichtigen.
    // Updated the screen to take into account the actions above.
    Display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

    // Extinction de la LED verte.
    // Grüne LED erlischt.
    // Green LED turns off.
    digitalWrite(USER_LED_GREEN, HIGH);
  }
 
  
  // Activation ou désactivation du tracker.
  // Aktivierung oder Deaktivierung des Trackers.
  // Activation or deactivation of the tracker.
  if((digitalRead(USER_BUTTON) == LOW) && PowerEnable && (digitalRead(USER_TOUCH) == HIGH))
  {
    // Permutation de l'état du tracker.
    // Tracker-Status tauschen.
    // Swap tracker state.
    TrackerEnable = !TrackerEnable;
    
    // Allumage de l'écran.
    // Bildschirmhintergrundbeleuchtung
    // Screen backlight.
    BacklightTime = millis();
    ShowTime = millis(); 
    digitalWrite(E_PAPER_BACKLIGHT, HIGH);

    // Effacement de l'écran et positionnement du curseur.
    // Bildschirm löschen und Cursor positionieren. 
    // Clearing the screen and positioning the cursor.
    Display.fillScreen(GxEPD_WHITE);
    Display.setCursor(0,12);

    // Affichage de la trame, du RSSI et du SNR à l'écran.
    // Anzeige von Frame, RSSI und SNR auf dem Bildschirm.
    // Displaying frame, RSSI and SNR on screen. 
    if(TrackerEnable) Display.println(F("Tracker ON"));
    else Display.println(F("Tracker OFF"));

    // Mise à jour de l'écran pour prendre en compte les actions ci-dessus.
    // Der Bildschirm wurde aktualisiert, um die oben genannten Aktionen zu berücksichtigen.
    // Updated the screen to take into account the actions above.
    Display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
  }


  // Demande d'émission d'une trame APRS au bout d'un cetain temps ou d'une certaine distance.
  // Anfrage zum Senden eines APRS-Frames nach einer bestimmten Zeit oder Entfernung.
  // Request to send an APRS frame after a certain time or distance.
  if(TrackerEnable && PowerEnable && ((Distance > TRANSMIT_DISTANCE) || ((millis() - TransmitTime) > (TRANSMIT_DELAY * 1000)))) Transmit = true;

  
  // Emission d'une trame APRS en cas de demande.
  // Aussenden eines APRS-Frames bei Anfrage.
  // Emission of an APRS frame in case of request.
  if(Transmit && !Receive && PowerEnable && Gps.location.isValid())
  {
    // Mémorisation de l'instant d'émission.
    // Speicherung des Moments der Emission.
    // Memorization of the moment of emission.
    TransmitTime = millis();
    
    // Retrait de la demande d'émission.
    // Rücknahme des Emissionsantrags.
    // Withdrawal of the emission request.
    Transmit = false;

    // Mise à jour de la latitude et la longitude d'émission.
    // Aktualisierung der Emissionsbreite und -länge.
    // Update of the emission latitude and longitude.
    TransmitLat = Gps.location.lat();
    TransmitLng = Gps.location.lng();
    
    // Allumage de la LED bleue.
    // Blaue LED leuchtet.
    // Blue LED lights up.
    digitalWrite(USER_LED_BLUE, LOW);

    // Allumage de l'écran.
    // Bildschirmhintergrundbeleuchtung
    // Screen backlight.
    BacklightTime = millis();
    ShowTime = millis();
    digitalWrite(E_PAPER_BACKLIGHT, HIGH);

    // Constitution de la trame APRS au format TNC2.
    // Aufbau des APRS-Frames im TNC2-Format.
    // Constitution of the APRS frame in TNC2 format.
    TNC2TxFrame = APRS_CALL;
    TNC2TxFrame += '>';
    TNC2TxFrame += APRS_RECIPIENT;
    TNC2TxFrame += ":=";
    TNC2TxFrame += LatitudeAprs(Gps.location.rawLat());
    TNC2TxFrame += APRS_OVERLAY;
    TNC2TxFrame += LongitudeAprs(Gps.location.rawLng());
    TNC2TxFrame += APRS_SYMBOL;
    TNC2TxFrame += APRS_COMMENT;
      
    // Effacement de l'écran et positionnement du curseur.
    // Bildschirm löschen und Cursor positionieren. 
    // Clearing the screen and positioning the cursor.
    Display.fillScreen(GxEPD_WHITE);
    Display.setCursor(0,12);

    // Affichage de la trame à l'écran.
    // Anzeige von Frame, RSSI und SNR auf dem Bildschirm.
    // Displaying frame, RSSI and SNR on screen. 
    Display.println(F("**** TX ****"));
    Display.println(TNC2TxFrame);

    // Ajout de l'en-tête LoRa au début de la trame TNC2.
    // LoRa-Header am Anfang des TNC2-Frames hinzugefügt.
    // Added the LoRa header at the start of the TNC2 frame.
    TNC2TxFrame = char(LORA_START_BYTE_3) + TNC2TxFrame;
    TNC2TxFrame = char(LORA_START_BYTE_2) + TNC2TxFrame;
    TNC2TxFrame = char(LORA_START_BYTE_1) + TNC2TxFrame;
      
    // Inhibition de l'interruption pour la réception radio LoRa.
    // Unterdrückung der Unterbrechung des LoRa-Funkempfangs.
    // Inhibition of interruption for LoRa radio reception.
    Interrupt = false;
    
    // Emission radio de la trame.
    // Funkübertragung des Frames.
    // Radio transmission of the frame.
    Radio.transmit(TNC2TxFrame);
 
    // Validation de l'interruption pour la réception radio LoRa.
    // Validierung der Unterbrechung für den LoRa-Funkempfang.
    // Validation of the interruption for LoRa radio reception.
    Interrupt = true;

    // Lancement de la réception radio LoRa.
    // Start des LoRa-Funkempfangs.
    // Launch of LoRa radio reception.
    State = Radio.startReceive();

    if(State != ERR_NONE)
    {
      Display.fillScreen(GxEPD_WHITE);
      Display.setCursor(0,12);

      Display.println(F("- Start Receive -"));
      Display.print(F("Failed : "));
      Display.println(State);
    }
    
    // Mise à jour de l'écran pour prendre en compte les actions ci-dessus.
    // Der Bildschirm wurde aktualisiert, um die oben genannten Aktionen zu berücksichtigen.
    // Updated the screen to take into account the actions above.
    Display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

    // Extinction de la LED bleue.
    // Ausschalten der blauen LED.
    // Turning off the blue LED.
    digitalWrite(USER_LED_BLUE, HIGH);
  }


  // Gestion du GPS.
  // GPS-Management.
  // GPS management.
  if((SerialGPS.available() > 0) && !Receive && PowerEnable)
  {
    // Encodage des signaux GPS reçus.
    // Kodierung der empfangenen GPS-Signale.
    // Encoding of received GPS signals.
    while(SerialGPS.available() > 0) Gps.encode(SerialGPS.read());

    // Calcul de la distance parcourue en mètres. La circonférence de la Terre est de 40000 Km. On utilise le théorème de Pythagore et la trigonométrie.
    // Berechnung der zurückgelegten Strecke in Metern. Der Erdumfang beträgt 40.000 km. Wir verwenden den Satz des Pythagoras und die Trigonometrie.
    // Calculation of the distance traveled in meters. The circumference of the Earth is 40,000 km. We use the Pythagorean theorem and trigonometry.
    if(Gps.location.isValid() && TransmitLat != 0.0 && TransmitLng != 0.0)
    Distance = 1000.0 * sqrt(pow((40000.0 / 360.0 * (Gps.location.lat() - TransmitLat)), 2) + pow((40000.0 * cos(Gps.location.lat()) / 360.0 * (Gps.location.lng() - TransmitLng)), 2));
  }


  // Gestion de l'affichage à l'écran de la dernière trame APRS reçue ou des coordonnées GPS.
  // Verwaltung der Bildschirmanzeige des zuletzt empfangenen APRS-Frames oder der GPS-Koordinaten.
  // Management of the on-screen display of the last APRS frame received or GPS coordinates.
  if(PowerEnable && ((millis() - ShowTime) > (SHOW_DELAY * 1000)))
  {
    // Permutation périodique entre l'affichage de la dernière trame APRS reçue et les coordonnées GPS.
    // Periodischer Wechsel zwischen der Anzeige des zuletzt empfangenen APRS-Frames und den GPS-Koordinaten.
    // Periodic switching between the display of the last APRS frame received and the GPS coordinates.
    if((millis() - SwapScreenTime) > (SWAP_SCREEN_PERIOD * 1000))
    {
      SwapScreenTime = millis();
      ViewGps = !ViewGps;
    }
    
    if(ViewGps)
    {
      if(Gps.location.isUpdated() || Gps.time.isUpdated())
      {
        // Effacement de l'écran et positionnement du curseur.
        // Bildschirm löschen und Cursor positionieren. 
        // Clearing the screen and positioning the cursor.
        Display.fillScreen(GxEPD_WHITE);
        Display.setCursor(0,12);

        // Affichage des coordonnées GPS.
        // Anzeige von GPS-Koordinaten.
        // Display of GPS coordinates.
        Display.println(F("**** GPS ****"));
        Display.print(F("Lat : "));
        Display.println(LatitudeAprs(Gps.location.rawLat()));
        Display.print(F("Lng : "));
        Display.println(LongitudeAprs(Gps.location.rawLng()));
        Display.print(F("Sats: "));
        Display.println(Gps.satellites.value());
        Display.print(F("Alt : "));
        Display.print(Gps.altitude.feet() / 3.2808);
        Display.println("M");
        Display.print(F("Time : "));
        Display.print(Gps.time.hour());
        Display.print(F(":"));
        Display.print(Gps.time.minute());
        Display.print(F(":"));
        Display.println(Gps.time.second());
        Display.print(F("Dist : "));
        Display.print(Distance);
        Display.println("M");
    
        // Mise à jour de l'écran pour prendre en compte les actions ci-dessus.
        // Der Bildschirm wurde aktualisiert, um die oben genannten Aktionen zu berücksichtigen.
        // Updated the screen to take into account the actions above.
        Display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false); 
      }
    }
    else
    {
      // Effacement de l'écran et positionnement du curseur.
      // Bildschirm löschen und Cursor positionieren. 
      // Clearing the screen and positioning the cursor.
      Display.fillScreen(GxEPD_WHITE);
      Display.setCursor(0,12);
     
      // Affichage de la dernière trame APRS reçue.
      // Anzeige des zuletzt empfangenen APRS-Frames.
      // Display of the last APRS frame received.
      Display.println(F("**** LAST ****"));
      Display.println(TNC2RxFrame.substring(3));
  
      Display.print(F("Length:\t"));
      Display.println(TNC2RxFrame.length());
  
      // Mise à jour de l'écran pour prendre en compte les actions ci-dessus.
      // Der Bildschirm wurde aktualisiert, um die oben genannten Aktionen zu berücksichtigen.
      // Updated the screen to take into account the actions above.
      Display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false); 
    }
  }


  // Gestion du bouton tactile.
  // Touch-Button-Verwaltung.
  // Touch button management.
  if((digitalRead(USER_BUTTON) == HIGH) && PowerEnable && (digitalRead(USER_TOUCH) == LOW))
  {
    // Allumage de l'écran.
    // Bildschirmhintergrundbeleuchtung
    // Screen backlight.
    BacklightTime = millis();
    digitalWrite(E_PAPER_BACKLIGHT, HIGH);

    // Pour un appuis long, on demande l'émission d'une trame APRS.
    // Bei langem Drücken fordern wir die Übertragung eines APRS-Frames an.
    // For a long press, we request the transmission of an APRS frame.
    if((millis() - TouchTime) > (TOUCH_DELAY * 1000)) Transmit = true;
  }
  else TouchTime = millis();
  

  // Extinction de l'éclairage de l'écran.
  // Bildschirmbeleuchtung ausschalten.
  // Turning off the screen lighting.
  if(((millis() - BacklightTime) > (BACKLIGHT_DELAY * 1000)) && (digitalRead(USER_BUTTON) == HIGH) && (digitalRead(USER_TOUCH) == HIGH) && PowerEnable) digitalWrite(E_PAPER_BACKLIGHT, LOW);

  
  // Arrêt.
  // Stoppen.
  // Power off.
  if((digitalRead(USER_BUTTON) == LOW) && PowerEnable && digitalRead(USER_TOUCH) == LOW)
  {
      Display.fillScreen(GxEPD_WHITE);
      Display.update();

      PowerEnable = false;
      digitalWrite(POWER_ENABLE, LOW);

      digitalWrite(E_PAPER_BACKLIGHT, LOW);
      
      while(digitalRead(USER_BUTTON) == LOW);
  }
  
  // Mise en marche.
  // Ein.
  // Power on.
  else if((digitalRead(USER_BUTTON) == LOW) && !PowerEnable)
  {
      digitalWrite(POWER_ENABLE, HIGH);
      PowerEnable = true;

      // Allumage de l'écran.
      BacklightTime = millis();
      digitalWrite(E_PAPER_BACKLIGHT, HIGH);

      Display.fillScreen(GxEPD_WHITE);
      Display.setCursor(0,12);
      Display.println(F("POWER ON"));
      Display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
      
      while(digitalRead(USER_BUTTON) == LOW);
  }
}


/**** DEFINITION DE LA FONCTION DE CONVERSION DE LA LATITUDE AU FORMAT APRS ****/
/**** DEFINITION DER KONVERTIERUNGSFUNKTION VON LATITUDE IN APRS-FORMAT ****/
/**** DEFINITION OF THE CONVERSION FUNCTION FROM LATITUDE TO APRS FORMAT ****/
String LatitudeAprs(RawDegrees Latitude)
{
  char Buffer[20];
  
  sprintf(Buffer, "%02d%05.2f", Latitude.deg, Latitude.billionths / 1000000000.0 * 60.0);
  
  String Str(Buffer);

  if(Latitude.negative) Str += 'S';
  else Str += 'N';
  
  return Str;
}


/**** DEFINITION DE LA FONCTION DE CONVERSION DE LA LONGITUDE AU FORMAT APRS ****/
/**** DEFINITION DER LÄNGENKONVERTIERUNGSFUNKTION IM APRS-FORMAT ****/
/**** DEFINITION OF THE LONGITUDE CONVERSION FUNCTION IN APRS FORMAT ****/
String LongitudeAprs(RawDegrees Longitude)
{
  char Buffer[20];

  sprintf(Buffer, "%03d%05.2f", Longitude.deg, Longitude.billionths / 1000000000.0 * 60.0);

  String Str(Buffer);

  if(Longitude.negative) Str += 'W';
  else Str += 'E';
  
  return Str;
}
