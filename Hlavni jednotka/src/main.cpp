#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ArduinoJson.h>
#include "credentials.h"

#define RXD2 16
#define TXD2 17

String hostname = "pocasi-loucka.eu";

AsyncWebServer server(80);
//-----------Proměnné-----------------------
const char *RadiacniStit_kompilace;
const char *Strecha_kompilace;
float RadiacniStit_Teplota_DS;
float RadiacniStit_Teplota_Si;
int RadiacniStit_vlhkost;

void setup()
{
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  //-----------------Bridge port
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

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

  //-----------------async web--------------- není nutno použít
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Ahoj! Tady bude jednou webové rozhraní s přehledem všech měřených hodnot... :-)"); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA

  server.begin();
  Serial.println("HTTP server started");
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
    }
    if (doc.containsKey("Strecha"))

    {
      Strecha_kompilace = doc["Kompilace"];
    }
  }
}
void WiFi_reconnect() //funkce na reconnect, pokud není připojeno, tak se co 30s snaží znova připojit
{
  unsigned long previousMillis;
  unsigned long interval = 30000;
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
  PrijemDat();
  WiFi_reconnect();
}