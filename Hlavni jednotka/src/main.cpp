#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h> // OTA
#include <ArduinoJson.h>
#include <WebSerial.h> //webserial
#include "SPIFFS.h"    //kvůli webu ve spiffs

#include "credentials.h" //prihlasováky + nastavení

#include <PubSubClient.h> //MQTT
WiFiClient espClient;
PubSubClient client(espClient);
int CasDat = 30; // cetnost reportu dat skrze MQTT
unsigned long PosledniOdeslaniDat;
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
float napetiBocnik = 0;
float napetiZatez = 0;
float proud = 0;
float prikon = 0;
//---------------DS18B20--------------------------
#include <OneWire.h> //dallas čidla
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 0
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature senzoryDS(&oneWire);
float temp100cm;
float temp50cm;
float temp20cm;
float temp10cm;
float temp5cm;
float tempPrizemni5cm;
float temp200cm;
//-----------Proměnné-----------------------
const char *RadiacniStit_kompilace;
float RadiacniStit_Teplota_DS;
float RadiacniStit_Teplota_Si;
int RadiacniStit_vlhkost;
float RadiacniStit_tlak;
float RadiacniStit_TeplotaBMP180;

const char *Strecha_kompilace;
float Strecha_winspeed;
int Strecha_srazky;
const char *Strecha_windir;
//------------------------------------
String hostname = "pocasi-loucka.eu"; // hostname na wifi

//-----------------Processor pro web-----------------

String processor(const String &var)
{

  if (var == "Komp_master")
  {
    return __DATE__ " " __TIME__;
  }
  else if (var == "napetiVstup")
  {
    return String(napetiVstup);
  }
  return String();
}

//----------WiFi - reconnect-----------
unsigned long previousMillis = 0;
unsigned long interval = 30000;

AsyncWebServer server(80);

void setup()
{
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
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  senzoryDS.begin(); //inicializace Dallas DS18B20

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(ssid, password);
  Serial.println("");

  /* // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }*/
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
  WebSerial.begin(&server);
  server.begin();
}

void PrijemDat()
{
  // Kontrola, zda přicházejí data

  if (Serial2.available())
  {
    // Stream & input;

    StaticJsonDocument<0> filter;
    filter.set(true);

    StaticJsonDocument<512> doc;

    String str;
    str = Serial2.readStringUntil('\n'); //čtení z webserial dokud není konec řádku
    WebSerial.println(str);              // výpis proměnné na web a nový řádek

    DeserializationError error = deserializeJson(doc, Serial2, DeserializationOption::Filter(filter));

    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    if (doc.containsKey("RadiacniStit"))

    {
      RadiacniStit_kompilace = doc["Kompilace"];
      RadiacniStit_Teplota_DS = doc["Temp_DS18B20"];
      RadiacniStit_Teplota_Si = doc["Temp_Si7021"];
      RadiacniStit_vlhkost = doc["Hum_Si7021"];
      RadiacniStit_tlak = doc["barometer"];
      RadiacniStit_TeplotaBMP180 = doc["Temp_BMP180"];
    }
    if (doc.containsKey("Strecha"))

    {
      Strecha_kompilace = doc["Kompilace"];
      Strecha_winspeed = doc["WinSpeed"];
      Strecha_srazky = doc["Rain"];
      Strecha_windir = doc["WinDir"];
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

void teplota()
{
  // adresy 1-wire čidel

  DeviceAddress Senzor100cm = {0x28, 0xAB, 0xA0, 0x77, 0x91, 0x11, 0x02, 0xB0};
  DeviceAddress Senzor50cm = {0x28, 0x89, 0x92, 0x77, 0x91, 0x11, 0x02, 0x6F};
  DeviceAddress Senzor20cm = {0x28, 0xA7, 0xE8, 0x77, 0x91, 0x09, 0x02, 0x80};
  DeviceAddress Senzor10cm = {0x28, 0xB9, 0x77, 0x77, 0x91, 0x14, 0x02, 0x75};
  DeviceAddress Senzor5cm = {0x28, 0xF4, 0xD0, 0x77, 0x91, 0x09, 0x02, 0x4D};
  DeviceAddress SenzorPrizemni5cm = {0x28, 0x30, 0xA4, 0x45, 0x92, 0x07, 0x02, 0x0B};
  DeviceAddress Senzor200cm = {0x28, 0x32, 0xD9, 0x35, 0x05, 0x00, 0x00, 0xCC};
  /*---------Proměnné-------------------------*/
  /*nastavení rozlišení čidel 9 bit  - 0,5°C
                        10 bit - 0,25°C
                        11 bit - 0,125°C
                        12 bit - 0,0625°C
  */

  senzoryDS.setResolution(10);
  senzoryDS.requestTemperatures();
  float tempZcidla100cm;

  float tempZcidla50cm;
  float tempZcidla20cm;
  float tempZcidla10cm;
  float tempZcidla5cm;
  float tempZcidlaPrizemni5cm;
  float tempZcidla200cm;
  /* 1-wire sekce */ // načtení informací ze všech čidel na daném pinu dle adresy a uložení do promněných

  tempZcidla100cm = senzoryDS.getTempC(Senzor100cm);
  tempZcidla50cm = senzoryDS.getTempC(Senzor50cm);
  tempZcidla20cm = senzoryDS.getTempC(Senzor20cm);
  tempZcidla10cm = senzoryDS.getTempC(Senzor10cm);
  tempZcidla5cm = senzoryDS.getTempC(Senzor5cm);
  tempZcidlaPrizemni5cm = senzoryDS.getTempC(SenzorPrizemni5cm);
  tempZcidla200cm = senzoryDS.getTempC(Senzor200cm);

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
  if ((-50 < tempZcidla200cm) && (tempZcidla200cm < 70))
    temp200cm = tempZcidla200cm;

  Serial.println("Načtení teploty z čidel");
}

void mqtt()
{
  if (millis() > PosledniOdeslaniDat + CasDat * 1000)
  {
    client.setServer(mqttServer, mqttPort);
    client.connect("pocasi-loucka.eu", mqttUser, mqttPassword);
    DynamicJsonDocument JSONencoder(1024);
    char buffer[256];
    JSONencoder["outTemp"] = RadiacniStit_Teplota_Si;
    JSONencoder["outHumidity"] = RadiacniStit_vlhkost;
    JSONencoder["windSpeed"] = Strecha_winspeed;
    JSONencoder["rain"] = Strecha_srazky;
    JSONencoder["windDir"] = Strecha_windir;

    JSONencoder["napetiVstup"] = napetiVstup;
    JSONencoder["proud"] = proud;
    JSONencoder["prikon"] = prikon;

    long rssi = WiFi.RSSI();
    JSONencoder["signal1"] = rssi;
    serializeJson(JSONencoder, buffer);

    Serial.println("Sending message to MQTT topic..");
    Serial.println(buffer);

    if (client.publish("meteostanice/pocasi-loucka.eu", buffer) == true)
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
  napetiVstup = ina219.getBusVoltage_V(); /// INA219
  prikon = ina219.getPower_mW();          // spotřeba v mW
  proud = ina219.getCurrent_mA();
  PrijemDat();
  mqtt();
  WiFi_reconnect();
}