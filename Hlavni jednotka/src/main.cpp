#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h> // OTA
#include <ArduinoJson.h>
#include "SPIFFS.h" //kvůli webu ve spiffs

#include "credentials.h" // prihlasovaky + nastaveni

#ifndef WEB_USERNAME
#define WEB_USERNAME mqttUser
#endif
#ifndef WEB_PASSWORD
#define WEB_PASSWORD mqttPassword
#endif

//--------------SD karta---------------------
#define SD_CS 5
#include "FS.h"
#include "SD.h"
#include <SPI.h>

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
bool appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return false;
  }

  const bool ok = file.print(message);
  Serial.println(ok ? "Message appended" : "Append failed");
  file.close();
  return ok;
}
//-------------MQTT-------------------------------------
#include <PubSubClient.h> //MQTT
WiFiClient espClient;
PubSubClient client(espClient);

//--------- hostname na wifi---------------------------
String hostname = "pocasi-loucka.eu";
//------------NTP + RTC-------------------------------
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <DS3231.h>
RTClib myRTC;

#define dec2bcd(dec_in) ((dec_in / 10) << 4) + (dec_in % 10)

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

byte tByte[0x07]; // holds the array from the NTP server
//-----------Serial 2 input -------------------
#define RXD2 16
#define TXD2 17
//-----------------ina219------------------------
#include <Wire.h>
#include <Adafruit_INA219.h>

// definování adresy senzoru
#define ADDR 0x40
// inicializace senzoru s nastavenou adresou z knihovny
Adafruit_INA219 ina219(ADDR);

float napetiVstup = 0;
float proud = 0;
float prikon = 0;
//---------------DS18B20--------------------------
#include <OneWire.h> //dallas čidla
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 0
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature senzoryDS(&oneWire);
float tempInside;

float temp100cm;
float temp50cm;
float temp20cm;
float temp10cm;
float temp5cm;
float tempPrizemni5cm;
//-----------Interní proměnné------------------------------
bool dataStrecha = false;
bool dataPlot = false;
bool novaData = false;
bool sdAvailable = false;
unsigned long casStrecha = 0;
unsigned long casPlot = 0;
unsigned long previousPowerMillis = 0;
unsigned long previousTemperatureMillis = 0;
unsigned long previousMqttAttempt = 0;
const unsigned long POWER_INTERVAL_MS = 1000;
const unsigned long TEMPERATURE_INTERVAL_MS = 10000;
const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;
String serial2Buffer;
//-----------Proměnné na data z meshe-----------------------
String Strecha_kompilace;
float Strecha_winspeed;
float Strecha_srazky;
float Strecha_srazkySD = 0.0f;
float Strecha_srazkyMQTT = 0.0f;
float Strecha_windir;
int Strecha_signal;
float NapetiWinDir;

String Plot_kompilace;
float Plot_tempDS18B20;
float Plot_tempBMP180;
float Plot_pressureBMP180;
float Plot_barometerBMP180;
float Plot_tempSHT31;
float Plot_humSHT31;
int Plot_signal;

//-----------------Processor pro web-----------------

String processor(const String &var)
{

  if (var == "Komp_master")
  {
    return __DATE__ " " __TIME__;
  }
  else if (var == "master_rssi")
  {
    return String(WiFi.RSSI());
  }
  else if (var == "esp_ram")
  {
    return String(ESP.getFreeHeap() / 1024);
  }
  else if (var == "uptime")
  {
    return String(millis() / 1000 / 60);
  }
  else if (var == "prikon")
  {
    return String(prikon);
  }
  else if (var == "napetiVstup")
  {
    return String(napetiVstup);
  }
  else if (var == "proud")
  {
    return String(proud);
  }
  else if (var == "master_teplota")
  {
    return String(tempInside);
  }
  else if (var == "Strecha_kompilace")
  {
    return String(Strecha_kompilace);
  }
  else if (var == "Strecha_winspeed")
  {
    return String(Strecha_winspeed);
  }
  else if (var == "Strecha_windir")
  {
    return String(Strecha_windir);
  }
  else if (var == "Strecha_signal")
  {
    return String(Strecha_signal);
  }
  else if (var == "NapetiWinDir")
  {
    return String(NapetiWinDir);
  }
  else if (var == "StavSD")
  {
    return sdAvailable ? "OK" : "ERROR";
  }
  else if (var == "Plot_signal")
  {
    return String(Plot_signal);
  }
  else if (var == "Plot_kompilace")
  {
    return String(Plot_kompilace);
  }
  else if (var == "Plot_tempDS18B20")
  {
    return String(Plot_tempDS18B20);
  }
  else if (var == "Plot_tempBMP180")
  {
    return String(Plot_tempBMP180);
  }
  else if (var == "Plot_tempSHT31")
  {
    return String(Plot_tempSHT31);
  }
  else if (var == "Plot_humSHT31")
  {
    return String(Plot_humSHT31);
  }
  else if (var == "Plot_pressureBMP180")
  {
    return String(Plot_pressureBMP180);
  }
  else if (var == "Plot_barometerBMP180")
  {
    return String(Plot_barometerBMP180);
  }
  else if (var == "tempPrizemni5cm")
  {
    return String(tempPrizemni5cm);
  }
  else if (var == "temp5cm")
  {
    return String(temp5cm);
  }
  else if (var == "temp10cm")
  {
    return String(temp10cm);
  }
  else if (var == "temp20cm")
  {
    return String(temp20cm);
  }
  else if (var == "temp50cm")
  {
    return String(temp50cm);
  }
  else if (var == "temp100cm")
  {
    return String(temp100cm);
  }
  else if (var == "casStrecha")
  {
    return casStrecha == 0 ? "N/A" : String((millis() - casStrecha) / 1000);
  }
  else if (var == "casPlot")
  {
    return casPlot == 0 ? "N/A" : String((millis() - casPlot) / 1000);
  }
  return String();
}

//----------WiFi - reconnect-----------
unsigned long previousMillis = 0;
unsigned long interval = 30000;

AsyncWebServer server(80);

void setup()
{
  Wire.begin(); //RTC
  ina219.begin();
  // nastavení kalibrace, k dispozici jsou 3 režimy
  // režim 32V a 2A má největší rozsahy, ale nejmenší přesnost
  //ina219.setCalibration_32V_2A();
  // režim 32V a 1A má lepší rozlišení průchozího proudu
  //ina219.setCalibration_32V_1A();
  // režim 16V a 400mA má nejlepší rozlišení proudu i napětí
  ina219.setCalibration_16V_400mA();
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  //-----------------Bridge port
  Serial2.begin(4800, SERIAL_8N1, RXD2, TXD2);

  senzoryDS.begin(); //inicializace Dallas DS18B20

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(ssid, password);
  Serial.println("");

  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Initialize SD card once. The status page never re-mounts it.
  sdAvailable = SD.begin(SD_CS);
  if (!sdAvailable || SD.cardType() == CARD_NONE)
  {
    sdAvailable = false;
    Serial.println("SD card unavailable");
  }
  else
  {
    Serial.println("SD card initialized");
    File file = SD.open("/data.txt");
    if (!file)
    {
      Serial.println("Creating /data.txt");
      writeFile(SD, "/data.txt", "dateTime,outTemp,outHumidity,barometer,windSpeed,rain,windDir,extraTemp1,extraTemp2,soilTemp1,soilTemp2,soilTemp3,soilTemp4,supplyVoltage\r\n");
    }
    else
    {
      file.close();
    }
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String(), false, processor); });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD))
                return request->requestAuthentication();
              if (!sdAvailable)
                return request->send(503, "text/plain", "SD card unavailable");
              request->send(SD, "/data.txt", "text/plain");
            });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              if (!request->authenticate(WEB_USERNAME, WEB_PASSWORD))
                return request->requestAuthentication();
              request->send(200, "text/plain", "Restarting");
              delay(250);
              ESP.restart();
            });

  AsyncElegantOTA.begin(&server, WEB_USERNAME, WEB_PASSWORD);
  server.begin();

  client.setServer(mqttServer, mqttPort);
  INA219napajeni();
  teplota();
}

void ntp2rtc()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Nacteni casu z NTP");
    timeClient.begin();

    // get the datetime from the NTP server
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();
    tByte[0] = (int)timeClient.getSeconds();
    tByte[1] = (int)timeClient.getMinutes();
    tByte[2] = (int)timeClient.getHours();
    tByte[3] = (int)timeClient.getDay();
    // create a struct to hold date, month and year values
    struct tm *ptm = gmtime((time_t *)&epochTime);
    tByte[4] = (int)ptm->tm_mday;
    tByte[5] = (int)ptm->tm_mon + 1;
    tByte[6] = (int)ptm->tm_year - 100;

    /* if the time stored in the DS3231 register does not match
   *  the time retrieved from the NTP server, update the DS3231
   *  register to the current time.
   */

    Wire.beginTransmission(0x68);
    // Set device to start read reg 0
    Wire.write(0x00);
    for (int idx = 0; idx < 7; idx++)
    {
      Wire.write(dec2bcd(tByte[idx]));
    }
    Wire.endTransmission();
  }
}

void processSerialMessage(const String &message)
{
  StaticJsonDocument<1024> doc;
  const DeserializationError error = deserializeJson(doc, message);
  if (error)
  {
    Serial.print("Bridge JSON error: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("Plot"))
  {
    Plot_kompilace = String(doc["Kompilace"] | "");
    Plot_tempDS18B20 = doc["tempDS18B20"] | Plot_tempDS18B20;
    Plot_tempBMP180 = doc["tempBMP180"] | Plot_tempBMP180;
    Plot_pressureBMP180 = doc["pressureBMP180"] | Plot_pressureBMP180;
    Plot_barometerBMP180 = doc["barometerBMP180"] | Plot_barometerBMP180;
    Plot_tempSHT31 = doc["tempSHT31"] | Plot_tempSHT31;
    Plot_humSHT31 = doc["humSHT31"] | Plot_humSHT31;
    Plot_signal = doc["Signal"] | Plot_signal;
    dataPlot = true;
    casPlot = millis();
    novaData = true;
  }

  if (doc.containsKey("Strecha"))
  {
    Strecha_kompilace = String(doc["Kompilace"] | "");
    Strecha_winspeed = doc["WinSpeed"] | Strecha_winspeed;
    Strecha_srazky = doc["Rain"] | 0.0f;
    Strecha_srazkySD += Strecha_srazky;
    Strecha_srazkyMQTT += Strecha_srazky;
    Strecha_windir = doc["WinDir"] | Strecha_windir;
    Strecha_signal = doc["Signal"] | Strecha_signal;
    NapetiWinDir = doc["NapetiWinDir"] | NapetiWinDir;
    dataStrecha = true;
    casStrecha = millis();
    novaData = true;
  }
}

void PrijemDat()
{
  while (Serial2.available())
  {
    const char c = static_cast<char>(Serial2.read());
    if (c == '\r')
      continue;

    if (c == '\n')
    {
      if (serial2Buffer.length() > 0)
      {
        processSerialMessage(serial2Buffer);
        serial2Buffer = "";
      }
      continue;
    }

    if (serial2Buffer.length() < 1024)
      serial2Buffer += c;
    else
    {
      Serial.println("Bridge frame too long, dropping");
      serial2Buffer = "";
    }
  }
}
void WiFi_reconnect() //funkce na reconnect, pokud není připojeno, tak se co 30s snaží znova připojit
{

  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (millis() - previousMillis >= interval))
  {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = millis();
  }
}
void INA219napajeni()
{
  napetiVstup = ina219.getBusVoltage_V(); /// INA219
  prikon = ina219.getPower_mW();          // spotřeba v mW
  proud = ina219.getCurrent_mA();
}
void teplota()

{
  // adresy 1-wire čidel

  DeviceAddress senzorMain = {0x28, 0xFF, 0x64, 0x1D, 0xF9, 0x87, 0xA8, 0xCC};
  DeviceAddress Senzor100cm = {0x28, 0xAB, 0xA0, 0x77, 0x91, 0x11, 0x02, 0xB0};
  DeviceAddress Senzor50cm = {0x28, 0x89, 0x92, 0x77, 0x91, 0x11, 0x02, 0x6F};
  DeviceAddress Senzor20cm = {0x28, 0xA7, 0xE8, 0x77, 0x91, 0x09, 0x02, 0x80};
  DeviceAddress Senzor10cm = {0x28, 0xB9, 0x77, 0x77, 0x91, 0x14, 0x02, 0x75};
  DeviceAddress Senzor5cm = {0x28, 0xF4, 0xD0, 0x77, 0x91, 0x09, 0x02, 0x4D};
  DeviceAddress SenzorPrizemni5cm = {0x28, 0x30, 0xA4, 0x45, 0x92, 0x07, 0x02, 0x0B};

  /*---------Proměnné-------------------------*/
  /*nastavení rozlišení čidel 9 bit  - 0,5°C
                        10 bit - 0,25°C
                        11 bit - 0,125°C
                        12 bit - 0,0625°C
  */

  senzoryDS.setResolution(9);
  senzoryDS.requestTemperatures();

  tempInside = senzoryDS.getTempC(senzorMain);
  float tempZcidla100cm = senzoryDS.getTempC(Senzor100cm);
  float tempZcidla50cm = senzoryDS.getTempC(Senzor50cm);
  float tempZcidla20cm = senzoryDS.getTempC(Senzor20cm);
  float tempZcidla10cm = senzoryDS.getTempC(Senzor10cm);
  float tempZcidla5cm = senzoryDS.getTempC(Senzor5cm);
  float tempZcidlaPrizemni5cm = senzoryDS.getTempC(SenzorPrizemni5cm);

  // validace teplot, teplota se do promněné uloží pouze, když je v rozsahu -50 - 70°C

  if ((-50 < tempZcidla100cm) && (tempZcidla100cm < 70))
    temp100cm = tempZcidla100cm;
  if ((-50 < tempZcidla50cm) && (tempZcidla50cm < 70))
    temp50cm = tempZcidla50cm;
  if ((-50 < tempZcidla20cm) && (tempZcidla20cm < 70))
    temp20cm = tempZcidla20cm;
  if ((-50 < tempZcidla10cm) && (tempZcidla10cm < 70))
    temp10cm = tempZcidla10cm;
  if ((-50 < tempZcidla5cm) && (tempZcidla5cm < 70))
    temp5cm = tempZcidla5cm;
  if ((-50 < tempZcidlaPrizemni5cm) && (tempZcidlaPrizemni5cm < 70))
    tempPrizemni5cm = tempZcidlaPrizemni5cm;
}
void logSDCard()
{
  if (!sdAvailable)
    return;

  static uint32_t lastLoggedSlot = UINT32_MAX;
  DateTime now = myRTC.now();

  if (now.minute() % 5 != 0)
    return;

  const uint32_t currentSlot = now.unixtime() / 300UL;
  if (currentSlot == lastLoggedSlot)
    return;

  String dataMessage = String(now.unixtime()) + "," +
                       String(Plot_tempDS18B20) + "," +
                       String(Plot_humSHT31) + "," +
                       String(Plot_barometerBMP180) + "," +
                       String(Strecha_winspeed) + "," +
                       String(Strecha_srazkySD) + "," +
                       String(Strecha_windir) + "," +
                       String(tempPrizemni5cm) + "," +
                       String(temp5cm) + "," +
                       String(temp10cm) + "," +
                       String(temp20cm) + "," +
                       String(temp50cm) + "," +
                       String(temp100cm) + "," +
                       String(napetiVstup) + "\r\n";

  Serial.print("Save data: ");
  Serial.println(dataMessage);
  if (appendFile(SD, "/data.txt", dataMessage.c_str()))
  {
    lastLoggedSlot = currentSlot;
    Strecha_srazkySD = 0.0f;
  }
}
void mqtt()
{
  if (!novaData || WiFi.status() != WL_CONNECTED)
    return;

  const bool sendPlot = dataPlot;
  const bool sendStrecha = dataStrecha;

  if (!client.connected())
  {
    const unsigned long now = millis();
    if (now - previousMqttAttempt < MQTT_RETRY_INTERVAL_MS)
      return;

    previousMqttAttempt = now;
    if (!client.connect("pocasi-loucka.eu", mqttUser, mqttPassword))
    {
      Serial.print("MQTT connect failed, state=");
      Serial.println(client.state());
      return;
    }
  }

  DynamicJsonDocument JSONencoder(512);
  if (sendPlot)
  {
    JSONencoder["outTemp"] = Plot_tempDS18B20;
    JSONencoder["outHumidity"] = Plot_humSHT31;
    JSONencoder["barometer"] = Plot_barometerBMP180;
    JSONencoder["signal3"] = Plot_signal;
  }
  if (sendStrecha)
  {
    JSONencoder["windSpeed"] = Strecha_winspeed;
    JSONencoder["rain"] = Strecha_srazkyMQTT;
    JSONencoder["windDir"] = Strecha_windir;
    JSONencoder["signal2"] = Strecha_signal;
  }

  JSONencoder["supplyVoltage"] = napetiVstup;
  JSONencoder["proud"] = proud;
  JSONencoder["prikon"] = prikon;
  JSONencoder["extraTemp1"] = tempPrizemni5cm;
  JSONencoder["extraTemp2"] = temp5cm;
  JSONencoder["soilTemp1"] = temp10cm;
  JSONencoder["soilTemp2"] = temp20cm;
  JSONencoder["soilTemp3"] = temp50cm;
  JSONencoder["soilTemp4"] = temp100cm;
  JSONencoder["signal1"] = WiFi.RSSI();

  String payload;
  serializeJson(JSONencoder, payload);
  Serial.println("Sending message to MQTT topic..");
  Serial.println(payload);

  if (client.publish("pocasi-loucka.eu", payload.c_str()))
  {
    Serial.println("Success sending message");
    if (sendPlot)
      dataPlot = false;
    if (sendStrecha)
    {
      dataStrecha = false;
      Strecha_srazkyMQTT = 0.0f;
    }
    novaData = dataPlot || dataStrecha;
  }
  else
  {
    Serial.println("Error sending message; keeping data pending");
    novaData = true;
  }
}

void loop()
{
  const unsigned long now = millis();

  if (now - previousPowerMillis >= POWER_INTERVAL_MS)
  {
    previousPowerMillis = now;
    INA219napajeni();
  }

  if (now - previousTemperatureMillis >= TEMPERATURE_INTERVAL_MS)
  {
    previousTemperatureMillis = now;
    teplota();
  }

  PrijemDat();
  logSDCard();
  mqtt();
  if (client.connected())
    client.loop();
  WiFi_reconnect();
}
