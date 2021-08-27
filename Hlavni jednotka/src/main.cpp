#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h> // OTA
#include <ArduinoJson.h>
#include "SPIFFS.h" //kvůli webu ve spiffs

#include "credentials.h" //prihlasováky + nastavení
#include <StreamUtils.h>// loging Serial2Serial

#include <PubSubClient.h> //MQTT
WiFiClient espClient;
PubSubClient client(espClient);
int CasDat = 30; // cetnost reportu dat skrze MQTT
unsigned long PosledniOdeslaniDat;

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
//-----------Interní proměnné------------------------------
bool dataStrecha;

//-----------Proměnné na data z meshe-----------------------
const char *Strecha_kompilace;
float Strecha_winspeed;
int Strecha_srazky;
float Strecha_windir;
int Strecha_signal;

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
  ina219.setCalibration_32V_1A();
  // režim 16V a 400mA má nejlepší rozlišení proudu i napětí
  //ina219.setCalibration_16V_400mA();
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  //-----------------Bridge port
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);

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

  //-----------------async web--------------- není nutno použít

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String(), false, processor); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
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

void PrijemDat()
{
  // Kontrola, zda přicházejí data

  if (Serial2.available())
  {
    // Stream & input;

    StaticJsonDocument<512> doc;

     ReadLoggingStream loggingStream(Serial2, Serial);
    DeserializationError error = deserializeJson(doc, loggingStream);
  /*  
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }*/

  //  deserializeJson(doc, Serial2);

    if (doc.containsKey("Strecha"))

    {
      Strecha_kompilace = doc["Kompilace"];
      Strecha_winspeed = doc["WinSpeed"];
      Strecha_srazky = doc["Rain"];
      Strecha_windir = doc["WinDir"];
      Strecha_signal = doc["Signal"];
      dataStrecha = true;
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

  /*---------Proměnné-------------------------*/
  /*nastavení rozlišení čidel 9 bit  - 0,5°C
                        10 bit - 0,25°C
                        11 bit - 0,125°C
                        12 bit - 0,0625°C
  */

  senzoryDS.setResolution(9);
  senzoryDS.requestTemperatures();

  tempInside = senzoryDS.getTempC(senzorMain);
}
void mqtt()
{
  if (millis() > PosledniOdeslaniDat + CasDat * 1000)
  {

    client.setServer(mqttServer, mqttPort);
    client.connect("pocasi-loucka.eu", mqttUser, mqttPassword);
    DynamicJsonDocument JSONencoder(1024);
    char buffer[256];
    if (dataStrecha == true)
    {
      JSONencoder["windSpeed"] = Strecha_winspeed;
      JSONencoder["rain"] = Strecha_srazky;
      JSONencoder["windDir"] = Strecha_windir;
      dataStrecha = false;
    }

    JSONencoder["napetiVstup"] = napetiVstup;
    JSONencoder["proud"] = proud;
    JSONencoder["prikon"] = prikon;

    long rssi = WiFi.RSSI();
    JSONencoder["signal1"] = rssi;
    serializeJson(JSONencoder, buffer);

    Serial.println("Sending message to MQTT topic..");
    Serial.println(buffer);

    if (client.publish("pocasi-loucka.eu", buffer) == true)
    {
      Serial.println("Success sending message");
    }
    else
    {
      Serial.println("Error sending message");
    }
    PosledniOdeslaniDat = millis();
  }
}

void loop()
{
  INA219napajeni();
  teplota();
  //  ntp2rtc();//aktualizace času v RTC
  PrijemDat();
  mqtt();
  WiFi_reconnect();
}