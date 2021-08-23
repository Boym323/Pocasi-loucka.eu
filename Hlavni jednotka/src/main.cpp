#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h> // OTA
#include <ArduinoJson.h>
#include "SPIFFS.h" //kvůli webu ve spiffs

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
float Strecha_windir;
int Strecha_signal;
//------------------------------------
String hostname = "pocasi-loucka.eu"; // hostname na wifi

//-----------------Processor pro web-----------------

String processor(const String &var)
{

  if (var == "Komp_master")
  {
    return __DATE__ " " __TIME__;
  }
  else if (var == "rssi")
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
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);

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
}

void PrijemDat()
{
  // Kontrola, zda přicházejí data

  if (Serial2.available())
  {
    // Stream & input;

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, Serial2);

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
      Strecha_signal = doc["Signal"];
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