#include "painlessMesh.h"
#include "credentials.h"

#include <ADS1115_WE.h>
#include <Wire.h>

ADS1115_WE adc = ADS1115_WE(0x48);

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
int casOdeslani = 30; //sekund
//weather sensors
const byte windSpeedPin = 12;
const byte rainPin = 14;
volatile unsigned int windcnt = 0;
volatile unsigned int raincnt = 0;
volatile unsigned long lastRainClick;
volatile unsigned long lastWindClick;
float windspeed;
float srazky;
float napetiWindDir;
String windDir;

IRAM_ATTR void WindSpeed()
{
  if ((millis() - lastWindClick > 16))
  { // debounce of sensor signal
    windcnt++;
    lastWindClick = millis();
  }
}

IRAM_ATTR void Rain()
{
  if ((millis() - lastRainClick > 500))
  { // debounce of sensor signal
    raincnt++;
    lastRainClick = millis();
  }
}
// Prototype
void receivedCallback(uint32_t from, String &msg);

size_t logServerId = 0;

// Send message to the logServer every 10 seconds

float readChannel(ADS1115_MUX channel)
{
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

Task nacteniDatCidla(casOdeslani * 1000, TASK_FOREVER, []()
                     {
                       //Calculate Wind Speed (klicks/interval * 2,4 kmh)
                       windspeed = ((windcnt / casOdeslani) * 2.4);
                       windcnt = 0;
                       //Calculate Rain
                       srazky = raincnt * (0.2794 / 10);
                       raincnt = 0;
                       //Calculate win direction
                       napetiWindDir = readChannel(ADS1115_COMP_0_GND);
                       if (napetiWindDir < 0.3216)
                         windDir = "112.5";
                       else if (napetiWindDir < 0.4092)
                         windDir = "67.5";
                       else if (napetiWindDir < 0.4545)
                         windDir = "90";
                       else if (napetiWindDir < 0.6166)
                         windDir = "157.5";
                       else if (napetiWindDir < 0.9016)
                         windDir = "135";
                       else if (napetiWindDir < 1.1936)
                         windDir = "202.5";
                       else if (napetiWindDir < 1.4029)
                         windDir = "180";
                       else if (napetiWindDir < 1.9821)
                         windDir = "22.5";
                       else if (napetiWindDir < 2.2527)
                         windDir = "45";
                       else if (napetiWindDir < 2.9268)
                         windDir = "247.5";
                       else if (napetiWindDir < 3.0769)
                         windDir = "225";
                       else if (napetiWindDir < 3.4314)
                         windDir = "337.5";
                       else if (napetiWindDir < 3.8372)
                         windDir = "0";
                       else if (napetiWindDir < 4.0407)
                         windDir = "292.5";
                       else if (napetiWindDir < 4.3324)
                         windDir = "315";
                       else if (napetiWindDir < 4.6154)
                         windDir = "270";
                       else
                         windDir = "0";
                     });

Task myLoggingTask(casOdeslani * 1000, TASK_FOREVER, []()
                   {
                     DynamicJsonDocument jsonBuffer(1024);
                     JsonObject msg = jsonBuffer.to<JsonObject>();
                     msg["Strecha"] = "Strecha";
                     msg["Kompilace"] = __DATE__ " " __TIME__;
                     msg["WinSpeed"] = windspeed;
                     msg["Rain"] = srazky;
                     msg["WinDir"] = windDir;
                     msg["NapetiWinDir"] = napetiWindDir;
                     msg["Signal"] = WiFi.RSSI();
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

  Wire.begin();
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setMeasureMode(ADS1115_SINGLE);
  pinMode(windSpeedPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), WindSpeed, FALLING);
  pinMode(rainPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rainPin), Rain, FALLING);

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6);
  mesh.onReceive(&receivedCallback);
  mesh.initOTAReceive("Strecha");

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
