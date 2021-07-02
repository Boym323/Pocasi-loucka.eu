#include "painlessMesh.h"
#include "credentials.h"

#include "Adafruit_Si7021.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#include <SFE_BMP180.h>
SFE_BMP180 pressure;
#define ALTITUDE 425 // nadmořská výška stanice
Adafruit_Si7021 sensor = Adafruit_Si7021();
double T, P, p0, a; // BMP180 T - teplota. P - tlak, P0 - tlak u hladiny moře

#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

Adafruit_Si7021 Si7021 = Adafruit_Si7021();
int casOdeslani = 20; //sekund

float tempDS18B20;
int humSi7021;
float tempSi7021;

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

// Prototype
void receivedCallback(uint32_t from, String &msg);

size_t logServerId = 0;

Task nacteniDatCidla(casOdeslani * 1000, TASK_FOREVER, []()
                     {
                       humSi7021 = Si7021.readHumidity();
                       tempSi7021 = Si7021.readTemperature();

                       DeviceAddress Teplota2m = {0x28, 0xFF, 0x64, 0x1D, 0xF9, 0x86, 0x3C, 0x78};
                       sensors.setResolution(Teplota2m, 11);
                       //Serial.print(sensors.getResolution(Teplota2m), DEC);
                       sensors.requestTemperatures();
                       tempDS18B20 = sensors.getTempC(Teplota2m);

                       char status;
                       status = pressure.startTemperature();
                       delay(status);
                       status = pressure.getTemperature(T);
                       status = pressure.startPressure(3);
                       delay(status);
                       status = pressure.getPressure(P, T);
                       p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
                     });
// Send message to the logServer every 10 seconds
Task myLoggingTask(casOdeslani * 1000, TASK_FOREVER, []()
                   {
                     DynamicJsonDocument jsonBuffer(1024);
                     JsonObject msg = jsonBuffer.to<JsonObject>();
                     msg["RadiacniStit"] = "RadiacniStit";
                     msg["Kompilace"] = __DATE__ " " __TIME__;
                     msg["Temp_DS18B20"] = tempDS18B20;
                     msg["Temp_Si7021"] = tempSi7021;
                     msg["Hum_Si7021"] = humSi7021;
                     msg["Temp_BMP180"] = T;
                     msg["barometer"] = p0;
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
  Si7021.begin();
  sensors.begin();
  Wire.begin();
  pressure.begin(); //BMP180

  Serial.begin(115200);

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6);
  mesh.onReceive(&receivedCallback);
  mesh.initOTAReceive("RadiacniStit");

  // Add the task to the your scheduler
  userScheduler.addTask(nacteniDatCidla);
  nacteniDatCidla.enable();
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
