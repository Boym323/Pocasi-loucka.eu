#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <ArduinoJson.h>

#define RXD2 16
#define TXD2 17

const char *ssid = "Home";
const char *password = "1234567890";
String hostname = "pocasi-loucka.eu";

AsyncWebServer server(80);
//------------------------------------------
//-----------Proměnné-----------------------
int Teplota;
int Vlhkost;
int Tlak;
int Slunce;
int Vitr;
int Srazky;

unsigned long previousMillis = 0;
unsigned long interval = 30000;

void setup()
{
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  //-----------------Bridge port
  Serial2.begin(4800, SERIAL_8N1, RXD2, TXD2);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //-----------------async web--------------- není nutno použít
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Ahoj! Tady bude jednou webové rozhraní s přehledem všech měřených hodnot... :-)"); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA

  server.begin();
  Serial.println("HTTP server started");
}

void PainlessMesh()
{
  // Kontrola, zda přicházejí data

  if (Serial2.available())
  {
    // Stream & input;

    StaticJsonDocument<0> filter;
    filter.set(true);

    StaticJsonDocument<512> doc;

    DeserializationError error = deserializeJson(doc, Serial2, DeserializationOption::Filter(filter));

    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    if (doc.containsKey("DeskaESP32"))

    {
      Slunce = doc["Slunce"]; // 64
      Srazky = doc["Srazky"]; // 167
      Vitr = doc["Vitr"];     // 91
      Serial.print("Slunce ");
      Serial.println(Slunce);
      Serial.print("Srazky ");
      Serial.println(Srazky);
      Serial.print("Vitr ");
      Serial.println(Vitr);
    }

    if (doc.containsKey("Lolin"))

    {
      Teplota = doc["Teplota"]; // 64
      Vlhkost = doc["Vlhkost"]; // 167
      Tlak = doc["Tlak"];       // 91
      Serial.print("Teplota ");
      Serial.println(Teplota);
      Serial.print("Vlhkost ");
      Serial.println(Vlhkost);
      Serial.print("Tlak ");
      Serial.println(Tlak);
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
void loop()
{
  AsyncElegantOTA.loop(); // OTA
  PainlessMesh();
  WiFi_reconnect();
}