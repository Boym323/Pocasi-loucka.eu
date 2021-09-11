#include "painlessMesh.h"
#include "credentials.h"

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS (0x76)
#define BMP280_CHIPID (0x58)
#define ALTITUDE 425 // nadmořská výška stanice
Adafruit_BMP280 bmp; // I2C

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

int casOdeslani = 3; //sekund

float tempDS18B20;

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

// Prototype
void receivedCallback(uint32_t from, String &msg);

size_t logServerId = 0;

// Send message to the logServer every 10 seconds
Task myLoggingTask(casOdeslani * 1000, TASK_FOREVER, []()
                   {
                     DeviceAddress Teplota2m = {0x28, 0x04, 0xB5, 0x79, 0xA2, 0x01, 0x03, 0xFB};
                     sensors.setResolution(Teplota2m, 10);
                     sensors.requestTemperatures();
                     tempDS18B20 = sensors.getTempC(Teplota2m);

                     DynamicJsonDocument jsonBuffer(1024);
                     JsonObject msg = jsonBuffer.to<JsonObject>();
                     msg["Plot"] = "Plot";
                     msg["Kompilace"] = __DATE__ " " __TIME__;
                     msg["tempDS18B20"] = tempDS18B20;
                     msg["tempBMP180"] = bmp.readTemperature();
                     msg["pressureBMP180"] = bmp.readPressure() / 100;
                     msg["barometerBMP180"] = bmp.seaLevelForAltitude(ALTITUDE, bmp.readPressure() / 100);
                     msg["tempSHT31"] = sht31.readTemperature();
                     msg["humSHT31"] = sht31.readHumidity();
                     String str;
                     serializeJson(msg, str);

                     if (logServerId == 0) // If we don't know the logServer yet
                       mesh.sendBroadcast(str);
                     else
                       mesh.sendSingle(logServerId, str);

                     // log to serial
                     serializeJson(msg, Serial);

                     Serial.printf("\n");
                   });

void setup()
{

  Serial.begin(115200);
  sht31.begin(0x44);
  sensors.begin();
  bmp.begin(BMP280_ADDRESS, BMP280_CHIPID);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6);
  mesh.onReceive(&receivedCallback);
  mesh.initOTAReceive("Plot");

  userScheduler.addTask(myLoggingTask);
  myLoggingTask.enable();
}

void loop()
{
  // it will run the user scheduler as well
  mesh.update();
}

void receivedCallback(uint32_t from, String &msg)
{
  Serial.printf("logClient: Received from %u msg=%s\n", from, msg.c_str());

  // Saving logServer
  DynamicJsonDocument jsonBuffer(1024 + msg.length());
  DeserializationError error = deserializeJson(jsonBuffer, msg);
  if (error)
  {
    Serial.printf("DeserializationError\n");
    return;
  }
  JsonObject root = jsonBuffer.as<JsonObject>();

  if (root.containsKey("topic"))
  {
    if (String("logServer").equals(root["topic"].as<String>()))
    {
      // check for on: true or false
      logServerId = root["nodeId"];
      Serial.printf("logServer detected!!!\n");
    }
    Serial.printf("Handled from %u msg=%s\n", from, msg.c_str());
  }
}
