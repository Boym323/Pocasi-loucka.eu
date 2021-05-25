#include <ArduinoJson.h>

#define RXD2 16
#define TXD2 17

//------------------------------------------
//-----------Proměnné-----------------------
int Teplota;
int Vlhkost;
int Tlak;
int Slunce;
int Vitr;
int Srazky;

void setup()
{
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  while (!Serial)
    continue;

  Serial2.begin(4800, SERIAL_8N1, RXD2, TXD2);

  // Initialize the "link" serial port
  // Use the lowest possible data rate to reduce error ratio
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


void loop()
{
  void PainlessMesh();
}