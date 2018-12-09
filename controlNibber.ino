#include <RtcDS3231.h>
#include <RCSwitch.h>
#include <EEPROM.h>
// Using Unified Sensor Libary for DHT22 && DHT11
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// RTC Configuration ////////////////////
RtcDS3231<TwoWire> RTC(Wire);
RtcDateTime currentDateTime;

// RC-Switch Configuration //////////////
RCSwitch RFSwitch = RCSwitch();

// DHT22 Configuration //////////////////
#define           MAX_TEMP        27
#define           MAX_HUMID       60

//#define           DHTPIN          2
//#define           DHTTYPE         DHT22
#define           DHTPIN          2
#define           DHTTYPE         DHT11

DHT_Unified       DHT_Sensor(DHTPIN, DHTTYPE);
sensors_event_t   DHT_Sensor_Event;

// STROM ////////////////////////////////
#define NIKS_STROM      0
#define LED_STROM       1
#define LUEFTER_STROM   2
#define ABLUFT_STROM    3

// ERROR ////////////////////////////////
#define RELAIS_ERROR      3
#define STEUERUNG_ERROR   2
#define RTC_ERROR         1
#define DHT22_ERROR       0
#define DHT11_ERROR       4

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const uint8_t ERROR_INDEX_MAX            = 10,
              MESSUNGS_INDEX_MAX         = 30,
              RELAIS_CHANGE_INDEX_MAX    = 20;

uint32_t steckdosenCodes[8] = {
  // ON ------ OFF
  14028656, 14383120,     // A
  14226996, 14341524,     // B
  14478156, 14341532,     // C
  14383122, 14226994      // D
};

bool relaisStatus[6];                    // [0, 1, 2, 3] Lichter; [4, 5] Lüfter
bool shortTimeActive            = false; // Standardmäßig in die Wachstumsphase

// Safety features
bool lightsAreCorrect           = false;
bool fansAreTurning             = false;
bool measurementsBelowMax       = false;

bool errorIndexOverflow         = false;
bool messungsIndexOverflow      = false;
bool relaisChangeIndexOverflow  = false;

uint8_t fanSpeedMeasurements  = 5;
uint8_t errorIndex            = 0;
uint8_t relaisChangeIndex     = 0;
uint8_t loopCounter           = 0;

uint8_t lightsOnStartHH[2]    = {  4, 10 }; // { longStart, shortStart }
uint8_t lightsOnEndHH[2]      = { 22, 22 }; // { longEnd,   shortEnd }
uint8_t fanTachoPins[4]       = {  2,  3, 4, 5 };

uint32_t CURRENT_TIME               = 0;
uint32_t FLOWERCYCLE_BEGIN          = 1544049215; // 05.12.2018 22:13:20

uint32_t fanSpeedMeasurementTimeOut = (4294967000 / (2 * fanSpeedMeasurements));
uint32_t fanSpeedOnThreshold        = 100;
uint32_t currentUnixTime            = 0;
uint32_t lastUnixTime               = 0;
uint32_t lightsSwitchUnixTime       = 0;
//uint32_t ledCooloffTime             = 60; // 1 Minute
// Nach ausschalten der LED's Lüfter für 1 Minute weiter drehen lassen um LED's abzukühlen

struct Messung {
  float           temperatur;
  float           feuchtigkeit;
  uint32_t        datum;                          // Unixtime
} messungsVerlauf[MESSUNGS_INDEX_MAX];            // 1 Minute Verlauf der Messungen.

struct RelaisChange {
  //bool            individualStatus[6];            // Optional [Status aller Relais für Analyse speichern]
  int16_t         relaisNumber;                   // Nummer des Relais welches geschalten wurde.
  bool            changedTo;                      // Ob das Relais an- oder ausgeschalten wurde.
  uint32_t        datum;                          // Unixtime
} relaisChangeVerlauf[RELAIS_CHANGE_INDEX_MAX];   // Letzen 20 Relais-Operationen speichern

struct Error {
  uint8_t         origin;                                    // [0] = DHT22, [1] = RTC, [2] = Steuerung, [3] = Relais
  uint16_t        errorNr;                                   // 404 = Growbox not found
  uint32_t        datum;                                    // Wann der Fehler aufgetreten ist;
} errorStack[ERROR_INDEX_MAX];                               // Errors

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// RELAIS //////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool getRelaisStatus(uint8_t pNumber) {
  // Aktuellen Status eines Relais auslesen.
  return true;
}

void updateAllRelaisStatus() {
  for (int i = 0; i < 6; ++i) {
    relaisStatus[i] = getRelaisStatus(i);
  }
}

void relaisChange(uint8_t pNumber, bool pSwitch) {
  if (pNumber > 3 || pNumber < 0) return; // Falsche Parameter

  RFSwitch.setProtocol(4);
  if (pSwitch) {
    RFSwitch.send(steckdosenCodes[pNumber * 2],     24);
  } else {
    RFSwitch.send(steckdosenCodes[pNumber * 2 + 1], 24);
  }
  addRelaisChange(pNumber, pSwitch);
  delay(100);
}

void relaisInitialize() {
  relaisChange(3, true); // Abluft anschalten
  for (int i = 0; i < 3; ++i) { // Alles Andere aus
    relaisChange(i, false);
  }
  delay(1000);
}

// STRUCT MANAGEMENT ///////////////////////////////////////////////////////////////////////////////////////////////////
void addMessung(float pTemperatur, float pFeuchtigkeit) {
  // Jede Messung um 1 nach hinten Verschieben
  for (int i = 28; i >= 0; ++i) {
    messungsVerlauf[i] = messungsVerlauf[i+1];
  }
  // Messung mit Index 0 eintragen
  messungsVerlauf[0].temperatur   = pTemperatur;
  messungsVerlauf[0].feuchtigkeit = pFeuchtigkeit;
  messungsVerlauf[0].datum        = CURRENT_TIME;
}

void addError(uint8_t pOrigin, uint8_t pErrorNr) {
  errorStack[errorIndex].origin        = pOrigin;
  errorStack[errorIndex].errorNr       = pErrorNr;
  errorStack[errorIndex].datum         = CURRENT_TIME;
  errorIndex++;
  if (errorIndex % (ERROR_INDEX_MAX - 1) == 0) errorIndex = 0;
}

void addRelaisChange(uint8_t pNumber, bool pSwitch) {
  relaisChangeVerlauf[relaisChangeIndex].relaisNumber = pNumber;
  relaisChangeVerlauf[relaisChangeIndex].changedTo = pSwitch;
  relaisChangeVerlauf[relaisChangeIndex].datum = CURRENT_TIME;
  relaisChangeIndex++;
  if (relaisChangeIndex % (RELAIS_CHANGE_INDEX_MAX - 1) == 0) relaisChangeIndex = 0;
}

// DHT22 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool readDHT22() {
  float feuchtigkeit  = 0.0f;
  float temperatur    = 0.0f;

  // Temperatur auslesen
  DHT_Sensor.temperature().getEvent(&DHT_Sensor_Event);
  if (!isnan(DHT_Sensor_Event.temperature)) {
    temperatur = DHT_Sensor_Event.temperature;
  } else { // Temperatur konnte nicht ausgelesen werden
    addError(DHT22_ERROR, 0);
    return false;
  }

  // Feuchtigkeit auslesen
  DHT_Sensor.humidity().getEvent(&DHT_Sensor_Event);
  if (!isnan(DHT_Sensor_Event.relative_humidity)) {
    feuchtigkeit = DHT_Sensor_Event.relative_humidity;
  } else { // Feuchtigkeit konnte nicht ausgelesen werden
    addError(DHT22_ERROR, 0);
    return false;
  }

  // Falls auslesen erfolgreich, Messung eintragen.
  addMessung(temperatur, feuchtigkeit);

  // Überprüfen ob gemessene Werte Grenzwerte überschreiten
  if (temperatur < MAX_TEMP && feuchtigkeit < MAX_HUMID) {
    measurementsBelowMax = true;
  } else { measurementsBelowMax = false }

  // Messung war erfolgreich
  return true;
}

bool readDHT11() {
  float feuchtigkeit  = 0.0f;
  float temperatur    = 0.0f;

  // Temperatur auslesen
  DHT_Sensor.temperature().getEvent(&DHT_Sensor_Event);
  if (!isnan(DHT_Sensor_Event.temperature)) {
    temperatur = DHT_Sensor_Event.temperature;
  } else { // Temperatur konnte nicht ausgelesen werden
    addError(DHT22_ERROR, 0);
    return false;
  }

  // Feuchtigkeit auslesen
  DHT_Sensor.humidity().getEvent(&DHT_Sensor_Event);
  if (!isnan(DHT_Sensor_Event.relative_humidity)) {
    feuchtigkeit = DHT_Sensor_Event.relative_humidity;
  } else { // Feuchtigkeit konnte nicht ausgelesen werden
    addError(DHT22_ERROR, 0);
    return false;
  }

  // Falls auslesen erfolgreich, Messung eintragen.
  addMessung(temperatur, feuchtigkeit);

  // Überprüfen ob gemessene Werte Grenzwerte überschreiten
  if (temperatur < MAX_TEMP && feuchtigkeit < MAX_HUMID) {
    measurementsBelowMax = true;
  } else { measurementsBelowMax = false }

  // Messung war erfolgreich
  return true;
}

// RTC /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setRtcDateTime() {
  // Check for Errors

  // Initialisiert die RTC mit der Uhrzeit, bzw dem Datum an dem dieser Sketch kompiliiert wurde
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  RTC.SetDateTime(compiled);
}

bool updateTime() {
  if (!RTC.IsDateTimeValid()) {
    addError(RTC_ERROR, 0);
  }

  currentDateTime = RTC.GetDateTime();
  currentUnixTime = currentDateTime.TotalSeconds() + 946684800;

  if (currentUnixTime < 1544008877) { // Wenn rtcTime kleiner als 05.12.2018 12:21 in |卐| DEUTSCHLAND |卐|
    return false;
  }

  if (currentUnixTime >= CURRENT_TIME) {
    CURRENT_TIME = currentUnixTime;
    return true; // Zeit wurde erfolgreich geupdated.
  } else {
    return false; // Etwas ist schief gelaufen.
  }
}

void RTCAlarmTriggered() {

}

// FAN CONTROL /////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int readFanRPM(uint8_t pFanNumber) {
  uint32_t fanSpeedSum = 0;

  for (int i = 0; i < fanSpeedMeasurements; ++i) {
    fanSpeedSum += pulseIn(pFanNumber, HIGH, fanSpeedMeasurementTimeOut);
    fanSpeedSum += pulseIn(pFanNumber, LOW , fanSpeedMeasurementTimeOut);
  }

  // Hoffentlich fanSpeed in RPM
  return (int) (1000000 / (fanSpeedSum / fanSpeedMeasurements)) * 60;
}

bool checkIfAllFansAreSpinning() {
  int fanRPM[4];

  for (int i = 0; i < 4; ++i) {
    fanRPM[i] = readFanRPM(fanTachoPins[i]);
  }

  for (int i = 0; i < 4; ++i) {
    if (fanRPM[i] < 100) return false;
  }

  return true;
}

// LIGHT CONTROL ///////////////////////////////////////////////////////////////////////////////////////////////////////
bool changeLights() { // Whether lights should be on or off dependent on current time and light cycle
  uint8_t currentHH = currentDateTime.Hour();

  // Ob lang oder kurz
  uint8_t shortTimeActiveArr = 0;
  if (shortTimeActive) shortTimeActiveArr++;

  //Licht Steuerung
  if (measurementsBelowMax                            &&    // Feuchtigkeit und Temperatur sind unter den Grenzwerten
      currentHH > lightsOnStartHH[shortTimeActiveArr] &&
      currentHH < lightsOnEndHH[shortTimeActiveArr]     ) {
    // LICHD MUSS ANN
    if (!fansAreTurning) {
      relaisChange(LUEFTER_STROM, true);
      return false; // Etwas ist schief gelaufen, Licht konnte nicht eingestellt werden
    }
    relaisChange(LED_STROM, true);
    return true; // Licht wurde erfolgreich eingestellt
  } else {
    // LICHD MUSS AUSS
    relaisChange(LED_STROM, false);
    relaisChange(LUEFTER_STROM, false);
    return true; // Licht wurde erfolgreich eingestellt
  }
}

// CONTROL STUFF ///////////////////////////////////////////////////////////////////////////////////////////////////////
void controlLoop() {
  checkIfAllFansAreSpinning();

  if (FLOWERCYCLE_BEGIN < CURRENT_TIME) {
    shortTimeActive = true;
  } else {
    shortTimeActive = false;
  }

  // Hat Protection mit eingebaut: Wenn lüfter aus sind und led's eingeschalten werden sollen
  // werden Lüfter eingeschalten, led's bleiben aus und Variable bleibt auf false.
  // In der nächsten loop wird geschaut ob die Lüfter sich schon schnell genug drehen um lichter einzuschalten
  // Sobald lüfter sich schnell genug drehen werden led's eingeschalten.
  // (changeLights() gibt true zurück, wenn alles korrekt eingestellt wurde und false wenn etwas noch nicht stimmt)
  if (!lightsAreCorrect) lightsAreCorrect = changeLights();
}

void loopCounterManagement() { // Measure DHT22 every 2 seconds
  //if (loopCounter % 2 == 1) readDHT22();
  //++loopCounter;
  //if (loopCounter % 2 == 0) loopCounter = 0;
  
  readDHT11();  
}

void printCurrentSystemState() {
  Serial.print("Errors: ");
  Serial.println(errorIndex);
  Serial.println("");

  Serial.print("Last read Temperature: ");
  Serial.println(messungsVerlauf[0].temperatur);
  Serial.println("");

  Serial.print("Last read Humidity: ");
  Serial.println(messungsVerlauf[0].feuchtigkeit);
  Serial.println("");

  Serial.print("Currently in short light phase?: ");
  Serial.println(shortTimeActive);
  Serial.println("");

  Serial.print("Current Time: ");
  Serial.println(currentUnixTime);
  Serial.println("");

  Serial.print("Current HH");
  Serial.println(currentDateTime.Hour());
  Serial.println("");

  Serial.print("Current MM");
  Serial.println(currentDateTime.Minute());
  Serial.println("");
}

bool

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Wire.begin();
  RTC.Begin();
  DHT_Sensor.begin();
  // // Falls RTC falsche Zeit anzeigt.
  // setRtcDateTime();
  relaisInitialize();
}

void loop() {
  delay(1000);             // no need to loop faster
  updateTime();            // update global time objects
  controlLoop();           // turn lights on or off according to time plan
  loopCounterManagement(); // read DHT22 every n-Seconds
  Serial.println(CURRENT_TIME);
}
