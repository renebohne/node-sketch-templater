{ "model" : "homeV2sim800", "board": "senseBox:samd:sb" }
/*
  senseBox:home - Citizen Sensingplatform
  Version: sim800_1.0.0
  Date: 2023-05-11
  Homepage: https://www.sensebox.de https://www.opensensemap.org
  Author: René Gern
  Note: Sketch for senseBox:home SIM800 MCU Edition
  Model: homeV2sim800
  Email: support@sensebox.de
  Code is in the public domain.
  https://github.com/sensebox/node-sketch-templater
*/

#include <Wire.h>
#include <SPI.h>

#include <senseBoxIO.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HDC1000.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME680.h>
#include <VEML6070.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <LTR329.h>
#include <SSLClient.h>
#include <Adafruit_DPS310.h> // http://librarymanager/All#Adafruit_DPS310
#include <sps30.h>
#include <TimeLib.h>


#define SerialMon Serial
#define SerialAT Serial3
#define TINY_GSM_MODEM_SIM800
//#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG SerialMon
#define GSM_PIN ""
#include <TinyGsmClient.h>
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif



#define TAs_NUM 1

static const unsigned char TA_DN0[] = {
    0x30, 0x3f, 0x31, 0x24, 0x30, 0x22, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x13,
    0x1b, 0x44, 0x69, 0x67, 0x69, 0x74, 0x61, 0x6c, 0x20, 0x53, 0x69, 0x67,
    0x6e, 0x61, 0x74, 0x75, 0x72, 0x65, 0x20, 0x54, 0x72, 0x75, 0x73, 0x74,
    0x20, 0x43, 0x6f, 0x2e, 0x31, 0x17, 0x30, 0x15, 0x06, 0x03, 0x55, 0x04,
    0x03, 0x13, 0x0e, 0x44, 0x53, 0x54, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20,
    0x43, 0x41, 0x20, 0x58, 0x33,
};

static const unsigned char TA_RSA_N0[] = {
    0xdf, 0xaf, 0xe9, 0x97, 0x50, 0x08, 0x83, 0x57, 0xb4, 0xcc, 0x62, 0x65,
    0xf6, 0x90, 0x82, 0xec, 0xc7, 0xd3, 0x2c, 0x6b, 0x30, 0xca, 0x5b, 0xec,
    0xd9, 0xc3, 0x7d, 0xc7, 0x40, 0xc1, 0x18, 0x14, 0x8b, 0xe0, 0xe8, 0x33,
    0x76, 0x49, 0x2a, 0xe3, 0x3f, 0x21, 0x49, 0x93, 0xac, 0x4e, 0x0e, 0xaf,
    0x3e, 0x48, 0xcb, 0x65, 0xee, 0xfc, 0xd3, 0x21, 0x0f, 0x65, 0xd2, 0x2a,
    0xd9, 0x32, 0x8f, 0x8c, 0xe5, 0xf7, 0x77, 0xb0, 0x12, 0x7b, 0xb5, 0x95,
    0xc0, 0x89, 0xa3, 0xa9, 0xba, 0xed, 0x73, 0x2e, 0x7a, 0x0c, 0x06, 0x32,
    0x83, 0xa2, 0x7e, 0x8a, 0x14, 0x30, 0xcd, 0x11, 0xa0, 0xe1, 0x2a, 0x38,
    0xb9, 0x79, 0x0a, 0x31, 0xfd, 0x50, 0xbd, 0x80, 0x65, 0xdf, 0xb7, 0x51,
    0x63, 0x83, 0xc8, 0xe2, 0x88, 0x61, 0xea, 0x4b, 0x61, 0x81, 0xec, 0x52,
    0x6b, 0xb9, 0xa2, 0xe2, 0x4b, 0x1a, 0x28, 0x9f, 0x48, 0xa3, 0x9e, 0x0c,
    0xda, 0x09, 0x8e, 0x3e, 0x17, 0x2e, 0x1e, 0xdd, 0x20, 0xdf, 0x5b, 0xc6,
    0x2a, 0x8a, 0xab, 0x2e, 0xbd, 0x70, 0xad, 0xc5, 0x0b, 0x1a, 0x25, 0x90,
    0x74, 0x72, 0xc5, 0x7b, 0x6a, 0xab, 0x34, 0xd6, 0x30, 0x89, 0xff, 0xe5,
    0x68, 0x13, 0x7b, 0x54, 0x0b, 0xc8, 0xd6, 0xae, 0xec, 0x5a, 0x9c, 0x92,
    0x1e, 0x3d, 0x64, 0xb3, 0x8c, 0xc6, 0xdf, 0xbf, 0xc9, 0x41, 0x70, 0xec,
    0x16, 0x72, 0xd5, 0x26, 0xec, 0x38, 0x55, 0x39, 0x43, 0xd0, 0xfc, 0xfd,
    0x18, 0x5c, 0x40, 0xf1, 0x97, 0xeb, 0xd5, 0x9a, 0x9b, 0x8d, 0x1d, 0xba,
    0xda, 0x25, 0xb9, 0xc6, 0xd8, 0xdf, 0xc1, 0x15, 0x02, 0x3a, 0xab, 0xda,
    0x6e, 0xf1, 0x3e, 0x2e, 0xf5, 0x5c, 0x08, 0x9c, 0x3c, 0xd6, 0x83, 0x69,
    0xe4, 0x10, 0x9b, 0x19, 0x2a, 0xb6, 0x29, 0x57, 0xe3, 0xe5, 0x3d, 0x9b,
    0x9f, 0xf0, 0x02, 0x5d,
};

static const unsigned char TA_RSA_E0[] = {
    0x01, 0x00, 0x01,
};

static const br_x509_trust_anchor TAs[] = {
    {
        { (unsigned char *)TA_DN0, sizeof TA_DN0 },
        BR_X509_TA_CA,
        {
            BR_KEYTYPE_RSA,
            { .rsa = {
                (unsigned char *)TA_RSA_N0, sizeof TA_RSA_N0,
                (unsigned char *)TA_RSA_E0, sizeof TA_RSA_E0,
            } }
        }
    },
};



#include <SSLClient.h>

// Uncomment the next line to get debugging messages printed on the Serial port
// Do not leave this enabled for long time use
#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG(str) SerialMon.println(str)
#define DEBUG_ARGS(str,str1) SerialMon.println(str,str1)
#define DEBUG2(str) SerialMon.print(str)
#define DEBUG_WRITE(c) SerialMon.write(c)
#else
#define DEBUG(str)
#define DEBUG_ARGS(str,str1)
#define DEBUG2(str)
#define DEBUG_WRITE(c)
#endif

/* ------------------------------------------------------------------------- */
/* ---------------------------------Metadata-------------------------------- */
/* ------------------------------------------------------------------------- */
/* SENSEBOX ID  : @@SENSEBOX_ID@@                                            */
/* SENSEBOX NAME: @@SENSEBOX_NAME@@                                          */
/* ------------------------------------------------------------------------- */
/* ------------------------------End of Metadata---------------------------- */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* ------------------------------Configuration------------------------------ */
/* ------------------------------------------------------------------------- */

// Interval of measuring and submitting values in seconds
const unsigned int postingInterval = 60e3;

// address of the server to send to
const char server[] PROGMEM = "@@INGRESS_DOMAIN@@";

// senseBox ID
const char SENSEBOX_ID[] PROGMEM = "@@SENSEBOX_ID@@";

// Number of sensors
// Change this number if you add or remove sensors
// do not forget to remove or add the sensors on opensensemap.org
static const uint8_t NUM_SENSORS = @@NUM_SENSORS@@;

// Connected sensors
@@SENSORS|toDefineWithSuffixPrefixAndKey~,_CONNECTED,sensorType@@

// Display enabled
// Uncomment the next line to get values of measurements printed on display
@@DISPLAY_ENABLED|toDefineDisplay@@

// Sensor SENSOR_IDs
@@SENSOR_IDS|toProgmem@@


const char apn[]      = "iot.telefonica.de";
const char gprsUser[] = "";
const char gprsPass[] = "";

TinyGsm modem(SerialAT);

// Choose the analog pin to get semi-random data from for SSL
// Pick a pin that's not connected or attached to a randomish voltage source
const int rand_pin = 2;

// Initialize the SSL client library
// We input a TinyGsmClient, our trust anchors, and the analog pin
TinyGsmClient base_client(modem);
SSLClient client(base_client, TAs, (size_t)TAs_NUM, rand_pin);

// Variables to measure the connection speed
unsigned long beginMicros, endMicros;
unsigned long byteCount = 0;
bool printWebData = true;  // set to false for better speed measurement


//Load sensors / instances
#ifdef HDC1080_CONNECTED
  Adafruit_HDC1000 HDC = Adafruit_HDC1000();
#endif
#ifdef BMP280_CONNECTED
  Adafruit_BMP280 BMP;
#endif
#ifdef TSL45315_CONNECTED
  bool lightsensortype = 0; //0 for tsl - 1 for ltr
  //settings for LTR sensor
  LTR329 LTR;
  unsigned char gain = 1;
  unsigned char integrationTime = 0;
  unsigned char measurementRate = 3;
#endif
#ifdef VEML6070_CONNECTED
  VEML6070 VEML;
#endif
#ifdef SMT50_CONNECTED
  #define SOILTEMPPIN @@SOIL_DIGITAL_PORT|digitalPortToPortNumber@@
  #define SOILMOISPIN @@SOIL_DIGITAL_PORT|digitalPortToPortNumber~1@@
#endif
#ifdef SOUNDLEVELMETER_CONNECTED
  #define SOUNDMETERPIN @@SOUND_METER_PORT|digitalPortToPortNumber@@
#endif
#ifdef BME680_CONNECTED
  Adafruit_BME680 BME;
#endif
#ifdef WINDSPEED_CONNECTED
  #define WINDSPEEDPIN @@WIND_DIGITAL_PORT|digitalPortToPortNumber@@
#endif
#ifdef SCD30_CONNECTED
  SCD30 SCD;
#endif
#ifdef DISPLAY128x64_CONNECTED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
#ifdef DPS310_CONNECTED
  Adafruit_DPS310 dps;
#endif
#ifdef SPS30_CONNECTED
  uint32_t auto_clean_days = 4;
  struct sps30_measurement m;
  int16_t ret;
  uint32_t auto_clean;
#endif


typedef struct measurement {
  const char *sensorId;
  float value;
} measurement;

measurement measurements[NUM_SENSORS];
uint8_t num_measurements = 0;

// buffer for sprintf
char buffer[750];

/* ------------------------------------------------------------------------- */
/* --------------------------End of Configuration--------------------------- */
/* ------------------------------------------------------------------------- */

unsigned long getTime() {
  

  int   year3    = 0;
  int   month3   = 0;
  int   day3     = 0;
  int   hour3    = 0;
  int   min3     = 0;
  int   sec3     = 0;
  float timezone = 0;
  for (int8_t i = 5; i; i--) {
    DEBUG(F("Requesting current network time"));
    if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,
                             &timezone)) {
      break;
    } else {
      DEBUG(F("Couldn't get network time, retrying in 5s."));
      delay(5000L);
    }
  }
  DEBUG(F("Retrieving time again as a string"));
  String time = modem.getGSMDateTime(DATE_FULL);
  DEBUG(F("Current Network Time: "));
  DEBUG2(time);

  const int offset = 1;   // Central European Time
  setTime(hour3,min3,sec3,day3,month3,year3);
  adjustTime(offset * SECS_PER_HOUR);

  unsigned long epoch = now();
  DEBUG(F("epoch: "));
  DEBUG2(epoch);
  return epoch;
}

void addMeasurement(const char *sensorId, float value) {
  measurements[num_measurements].sensorId = sensorId;
  measurements[num_measurements].value = value;
  num_measurements++;
}

void writeMeasurementsToClient() {
  // iterate throug the measurements array
  for (uint8_t i = 0; i < num_measurements; i++) {
    sprintf_P(buffer, PSTR("%s,%9.2f\n"), measurements[i].sensorId,
              measurements[i].value);

    // transmit buffer to client
    client.print(buffer);
    DEBUG(buffer);
  }

  // reset num_measurements
  num_measurements = 0;
}

void submitValues() {
  // close any connection before send a new request.
  // This will free the socket
  if (client.connected()) {
    client.stop();
    delay(1000);
  }

   if (!modem.isGprsConnected()) {
    connectModem();
  }

  bool connected = false;
  char _server[strlen_P(server)];
  strcpy_P(_server, server);
  for (uint8_t timeout = 2; timeout != 0; timeout--) {
    DEBUG(F("connecting..."));
    connected = client.connect(_server, 443);
    if (connected == true) {
      DEBUG(F("Connection successful, transferring..."));
      // construct the HTTP POST request:
      sprintf_P(buffer,
                PSTR("POST /boxes/%s/data HTTP/1.1\nAuthorization: @@ACCESS_TOKEN@@\nHost: %s\nContent-Type: "
                     "text/csv\nConnection: close\nContent-Length: %i\n\n"),
                SENSEBOX_ID, server, num_measurements * 35);
      DEBUG(buffer);

      // send the HTTP POST request:
      client.print(buffer);

      // send measurements
      writeMeasurementsToClient();

      // send empty line to end the request
      client.println();

      uint16_t timeout = 0;
      // allow the response to be computed

      while (timeout <= 5000) {
        delay(10);
        timeout = timeout + 10;
        if (client.available()) {
          break;
        }
      }

      while (client.available()) {
        char c = client.read();
        DEBUG_WRITE(c);
        // if the server's disconnected, stop the client:
        if (!client.connected()) {
          DEBUG();
          DEBUG(F("disconnecting from server."));
          client.stop();
          break;
        }
      }

      DEBUG(F("done!"));

      // reset number of measurements
      num_measurements = 0;
      break;
    }
    delay(1000);
  }

  if (connected == false) {
    // Reset durchführen
    DEBUG(F("connection failed. Restarting System."));
    delay(5000);
    noInterrupts();
    NVIC_SystemReset();
    while (1)
      ;
  }
}

void checkI2CSensors() {
  byte error;
  int nDevices = 0;
  byte sensorAddr[] = {41, 56, 57, 64, 97, 118};
  DEBUG(F("\nScanning..."));
  for (int i = 0; i < sizeof(sensorAddr); i++) {
    Wire.beginTransmission(sensorAddr[i]);
    error = Wire.endTransmission();
    if (error == 0) {
      nDevices++;
      switch (sensorAddr[i])
      {
        case 0x29:
          DEBUG(F("TSL45315 found."));
          break;
        case 0x38: // &0x39
          DEBUG(F("VEML6070 found."));
          break;
        case 0x40:
          DEBUG(F("HDC1080 found."));
          break;
        case 0x76:
        #ifdef BMP280_CONNECTED
          DEBUG("BMP280 found.");
        #elif defined(BME680_CONNECTED)
          DEBUG("BME680 found.");
        #else
          DEBUG("DPS310 found.");
        #endif
          break;
        case 0x61:
          DEBUG("SCD30 found.");
          break;
      }
    }
    else if (error == 4)
    {
      DEBUG2(F("Unknown error at address 0x"));
      if (sensorAddr[i] < 16)
        DEBUG2(F("0"));
      DEBUG_ARGS(sensorAddr[i], HEX);
    }
  }
  if (nDevices == 0) {
    DEBUG(F("No I2C devices found.\nCheck cable connections and press Reset."));
    while(true);
  } else {
    DEBUG2(nDevices);
    DEBUG(F(" sensors found.\n"));
  }
}

bool connectModem()
{
    // The GPRSBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);

  DEBUG("Waiting for network...");
  if (!modem.waitForNetwork(600000L, true)) {
    return false;
  }

  if (modem.isNetworkConnected()) { DEBUG("Network connected"); }

  DEBUG2(F("Connecting to "));
  DEBUG(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    return false;
  }

  return modem.isGprsConnected();
}

void setup() {
  // Initialize serial and wait for port to open:
  #ifdef ENABLE_DEBUG
    Serial.begin(9600);
  #endif
  delay(5000);

  pinMode(PIN_XB1_CS, OUTPUT);
  digitalWrite(PIN_XB1_CS, LOW);
  senseBoxIO.powerAll();

  delay(1000);

#ifdef DISPLAY128x64_CONNECTED
  DEBUG2(F("enable display..."));
  delay(2000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.display();
  delay(100);
  display.clearDisplay();
  DEBUG(F("done."));
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.println("senseBox:");
  display.println("home\n");
  display.setTextSize(1);
  display.println("Version SIM800 modem");
  display.setTextSize(2);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Connecting to:");
  display.println();
  display.println(apn);
  display.setTextSize(1);
  display.display();
#endif


  SerialAT.begin(115200); // SIM800 modem
  delay(10);
  digitalWrite(PIN_XB1_CS, LOW);//logic inverted by T1 (GPRSBee) and inverted by T1T1 (sensebox) -> inverted inverted = not inverted
  delay(1100);
  digitalWrite(PIN_XB1_CS, HIGH);
  delay(100);

  DEBUG2(F("Initializing modem..."));
  modem.restart();

  // Check GPRSBee status
   String modemInfo = modem.getModemInfo();
  DEBUG2(F("Modem Info: "));
  DEBUG(modemInfo);
  
  // attempt to connect to Mobile network:
  DEBUG2(F("Attempting to connect to APN: "));
  DEBUG(apn);
  bool status = connectModem();
  while (status != true) {
#ifdef DISPLAY128x64_CONNECTED
    display.print(".");
    display.display();
#endif  
    status = modem.waitForNetwork();
  }

  #ifdef ENABLE_DEBUG
    // init I2C/wire library
    Wire.begin();
    checkI2CSensors();
  #endif

  // Sensor initialization
  DEBUG(F("Initializing sensors..."));
  #ifdef HDC1080_CONNECTED
    HDC.begin();
  #endif
  #ifdef BMP280_CONNECTED
    BMP.begin(0x76);
  #endif
  #ifdef VEML6070_CONNECTED
    VEML.begin();
    delay(500);
  #endif
  #ifdef TSL45315_CONNECTED
    Lightsensor_begin();
  #endif
  #ifdef BME680_CONNECTED
    BME.begin(0x76);
    BME.setTemperatureOversampling(BME680_OS_8X);
    BME.setHumidityOversampling(BME680_OS_2X);
    BME.setPressureOversampling(BME680_OS_4X);
    BME.setIIRFilterSize(BME680_FILTER_SIZE_3);
  #endif
  #ifdef SCD30_CONNECTED
    Wire.begin();
    SCD.begin();
  #endif
  #ifdef DISPLAY128x64_CONNECTED
  display.clearDisplay();
  display.setCursor(30, 28);
  display.setTextSize(2);
  display.print("Ready!");
  display.display();
  #endif
  #ifdef DPS310_CONNECTED
    dps.begin_I2C(0x76);
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  #endif
  #ifdef SPS30_CONNECTED
    sensirion_i2c_init();
    ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
    ret = sps30_start_measurement();
  #endif
  DEBUG(F("Initializing sensors done!"));
  DEBUG(F("Starting loop in 3 seconds."));
  delay(3000);
}


void loop() {
  DEBUG(F("Starting new measurement..."));
#ifdef DISPLAY128x64_CONNECTED
  long displayTime = 5000;
  int page = 0;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.println("Uploading new measurement... ");
  display.display();
#endif
  // capture loop start timestamp
  unsigned long start = millis();

  //-----Temperature-----//
  //-----Humidity-----//
#ifdef HDC1080_CONNECTED
  addMeasurement(HDC1080_TEMPERSENSOR_ID, HDC.readTemperature());
  delay(200);
  addMeasurement(HDC1080_RELLUFSENSOR_ID, HDC.readHumidity());
#endif

  //-----Pressure-----//
#ifdef BMP280_CONNECTED
  float pressure;
  pressure = BMP.readPressure() / 100;
  addMeasurement(BMP280_LUFTDRSENSOR_ID, pressure);
#endif

  //-----Lux-----//
#ifdef TSL45315_CONNECTED
  addMeasurement(TSL45315_BELEUCSENSOR_ID, Lightsensor_getIlluminance());
#endif

  //-----UV intensity-----//
#ifdef VEML6070_CONNECTED
  addMeasurement(VEML6070_UVINTESENSOR_ID, VEML.getUV());
#endif

  //-----Soil Temperature & Moisture-----//
#ifdef SMT50_CONNECTED
  float voltage = analogRead(SOILTEMPPIN) * (3.3 / 1024.0);
  float soilTemperature = (voltage - 0.5) * 100;
  addMeasurement(SMT50_BODENTSENSOR_ID, soilTemperature);
  voltage = analogRead(SOILMOISPIN) * (3.3 / 1024.0);
  float soilMoisture = (voltage * 50) / 3;
  addMeasurement(SMT50_BODENFSENSOR_ID, soilMoisture);
#endif

  //-----dB(A) Sound Level-----//
#ifdef SOUNDLEVELMETER_CONNECTED
  float v = analogRead(SOUNDMETERPIN) * (3.3 / 1024.0);
  float decibel = v * 50;
  addMeasurement(SOUNDLEVELMETER_LAUTSTSENSOR_ID, decibel);
#endif

  //-----BME680-----//
#ifdef BME680_CONNECTED
  BME.setGasHeater(0, 0);
  float gasResistance;
  if ( BME.performReading()) {
    addMeasurement(BME680_LUFTTESENSOR_ID, BME.temperature - 1);
    addMeasurement(BME680_LUFTFESENSOR_ID, BME.humidity);
    addMeasurement(BME680_ATMLUFSENSOR_ID, BME.pressure / 100);
  }
  BME.setGasHeater(320, 150); // 320*C for 150 ms
  if ( BME.performReading()) {
      gasResistance = BME.gas_resistance / 1000.0;
       addMeasurement(BME680_VOCSENSOR_ID, gasResistance);
  }
#endif

  //-----Wind speed-----//
#ifdef WINDSPEED_CONNECTED
  float voltageWind = analogRead(WINDSPEEDPIN) * (3.3 / 1024.0);
  float windspeed = 0.0;
  if (voltageWind >= 0.018) {
    float poly1 = pow(voltageWind, 3);
    poly1 = 17.0359801998299 * poly1;
    float poly2 = pow(voltageWind, 2);
    poly2 = 47.9908168343362 * poly2;
    float poly3 = 122.899677524413 * voltageWind;
    float poly4 = 0.657504127272728;
    windspeed = poly1 - poly2 + poly3 - poly4;
    windspeed = windspeed * 0.2777777777777778; //conversion in m/s
  }
  addMeasurement(WINDSPEED_WINDGESENSOR_ID, windspeed);
#endif

  //-----CO2-----//
#ifdef SCD30_CONNECTED
  addMeasurement(SCD30_CO2SENSOR_ID, SCD.getCO2());
#endif

  //-----DPS310 Pressure-----//
  #ifdef DPS310_CONNECTED
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    addMeasurement(DPS310_LUFTDRSENSOR_ID, pressure_event.pressure);
  #endif

  #ifdef SPS30_CONNECTED
    ret = sps30_read_measurement(&m);
    addMeasurement(SPS30_PM1SENSOR_ID, m.mc_1p0);
    addMeasurement(SPS30_PM25SENSOR_ID, m.mc_2p5);
    addMeasurement(SPS30_PM4SENSOR_ID, m.mc_4p0);
    addMeasurement(SPS30_PM10SENSOR_ID, m.mc_10p0);
  #endif

  DEBUG(F("Submit values"));
  submitValues();

  // schedule next round of measurements
  for (;;) {
    unsigned long now = millis();
    unsigned long elapsed = now - start;
#ifdef DISPLAY128x64_CONNECTED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);
    switch (page)
    {
      case 0:
        // HDC & BMP
        display.setTextSize(2);
        display.setTextColor(BLACK, WHITE);
        display.println(F("HDC&BMP"));
        display.setTextColor(WHITE, BLACK);
        display.setTextSize(1);
        display.print(F("Temp:"));
#ifdef HDC1080_CONNECTED
        display.println(HDC.readTemperature());
#else
        display.println(F("not connected"));
#endif
        display.println();
        display.print(F("Humi:"));
#ifdef HDC1080_CONNECTED
        display.println(HDC.readHumidity());
#else
        display.println(F("not connected"));
#endif
        display.println();
        display.print(F("Press.:"));
#ifdef BMP280_CONNECTED
        display.println(BMP.readPressure() / 100);
#else
        display.println(F("not connected"));
#endif
        break;
      case 1:
        // TSL/VEML
        display.setTextSize(2);
        display.setTextColor(BLACK, WHITE);
        display.println(F("TSL&VEML"));
        display.setTextColor(WHITE, BLACK);
        display.println();
        display.setTextSize(1);
        display.print(F("Lux:"));
#ifdef TSL45315_CONNECTED
        display.println(Lightsensor_getIlluminance());
#else
        display.println(F("not connected"));
#endif
        display.println();
        display.print("UV:");
#ifdef VEML6070_CONNECTED
        display.println(VEML.getUV());
#else
        display.println(F("not connected"));
#endif
        break;
      case 2:
        // SPS30_CONNECTED
        display.setTextSize(2);
        display.setTextColor(BLACK, WHITE);
        display.println(F("PM1&PM2.5"));
        display.setTextColor(WHITE, BLACK);
        display.println();
        display.setTextSize(1);
        display.print(F("PM1:"));

        #ifdef SPS30_CONNECTED
          display.println(m.mc_1p0);
        #else
          display.println(F("not connected"));
        #endif

        display.print(F("PM.25:"));
        #ifdef SPS30_CONNECTED
          display.println(m.mc_2p5);
        #else
          display.println(F("not connected"));
        #endif

        break;
      case 3:
        // SPS30_CONNECTED
        display.setTextSize(2);
        display.setTextColor(BLACK, WHITE);
        display.println(F("PM4&PM10"));
        display.setTextColor(WHITE, BLACK);
        display.println();
        display.setTextSize(1);
        display.print(F("PM4:"));

        #ifdef SPS30_CONNECTED
          display.println(m.mc_4p0);
        #else
          display.println(F("not connected"));
        #endif
          display.print(F("PM10:"));
        #ifdef SPS30_CONNECTED
          display.println(m.mc_10p0);
        #else
          display.println(F("not connected"));
        #endif
        break;
      case 4:
        // SMT, SOUND LEVEL , BME
        display.setTextSize(2);
        display.setTextColor(BLACK, WHITE);
        display.println(F("Soil"));
        display.setTextColor(WHITE, BLACK);
        display.println();
        display.setTextSize(1);
        display.print(F("Temp:"));
#ifdef SMT50_CONNECTED
        display.println(soilTemperature);
#else
        display.println(F("not connected"));
#endif
        display.println();
        display.print(F("Moist:"));
#ifdef SMT50_CONNECTED
        display.println(soilMoisture);
#else
        display.println(F("not connected"));
#endif

        break;
      case 5:
        // WINDSPEED SCD30
        display.setTextSize(2);
        display.setTextColor(BLACK, WHITE);
        display.println(F("Wind&SCD30"));
        display.setTextColor(WHITE, BLACK);
        display.println();
        display.setTextSize(1);
        display.print(F("Speed:"));
#ifdef WINDSPEED_CONNECTED
        display.println(windspeed);
#else
        display.println(F("not connected"));
#endif
        display.println();
        display.print(F("SCD30:"));
#ifdef SCD30_CONNECTED
        display.println(SCD.getCO2());
#else
        display.println(F("not connected"));
#endif
        break;
      case 6:
          // SMT, SOUND LEVEL , BME
        display.setTextSize(2);
        display.setTextColor(BLACK, WHITE);
        display.println(F("Sound&BME"));
        display.setTextColor(WHITE, BLACK);
        display.println();
        display.setTextSize(1);
        display.print(F("Sound:"));
#ifdef SOUNDLEVELMETER_CONNECTED
        display.println(decibel);
#else
        display.println(F("not connected"));
#endif
        display.println();
        display.print(F("Gas:"));
#ifdef BME680_CONNECTED
        display.println(gasResistance);
#else
        display.print(F("not connected"));
#endif

        break;
    }
    display.display();
    if (elapsed >= displayTime)
    {
      if (page == 4)
      {
        page = 0;
      }
      else
      {
        page += 1;
      }
      displayTime += 5000;
    }
#endif
    if (elapsed >= postingInterval)
      return;
  }
}

int read_reg(byte address, uint8_t reg)
{
  int i = 0;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address, (uint8_t)1);
  delay(1);
  if(Wire.available())
    i = Wire.read();

  return i;
}

void write_reg(byte address, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

#ifdef TSL45315_CONNECTED
void Lightsensor_begin()
{
  Wire.begin();
  unsigned int u = 0;
  DEBUG(F("Checking lightsensortype"));
  u = read_reg(0x29, 0x80 | 0x0A); //id register
  if ((u & 0xF0) == 0xA0)            // TSL45315
  {
    DEBUG(F("TSL45315"));
    write_reg(0x29, 0x80 | 0x00, 0x03); //control: power on
    write_reg(0x29, 0x80 | 0x01, 0x02); //config: M=4 T=100ms
    delay(120);
    lightsensortype = 0; //TSL45315
  }
  else
  {
    DEBUG(F("LTR329"));
    LTR.begin();
    LTR.setControl(gain, false, false);
    LTR.setMeasurementRate(integrationTime, measurementRate);
    LTR.setPowerUp(); //power on with default settings
    delay(10); //Wait 10 ms (max) - wakeup time from standby
    lightsensortype = 1;                     //
  }
}

unsigned int Lightsensor_getIlluminance()
{
  unsigned int lux = 0;
  if (lightsensortype == 0) // TSL45315
  {
    unsigned int u = (read_reg(0x29, 0x80 | 0x04) << 0);  //data low
    u |= (read_reg(0x29, 0x80 | 0x05) << 8); //data high
    lux = u * 4; // calc lux with M=4 and T=100ms
  }
  else if (lightsensortype == 1) //LTR-329ALS-01
  {
    delay(100);
    unsigned int data0, data1;
    for (int i = 0; i < 5; i++) {
      if (LTR.getData(data0, data1)) {
        if(LTR.getLux(gain, integrationTime, data0, data1, lux));
        else DEBUG(F("LTR sensor saturated"));
        if(lux > 0) break;
        else delay(10);
      }
      else {
        DEBUG2(F("LTR getData error "));
        byte error = LTR.getError();
        DEBUG(error);
      }
    }
  }
  return lux;
}
#endif
