#include "painlessMesh.h"
#include "credentials.h"


Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

// Prototype
void receivedCallback(uint32_t from, String &msg);

size_t logServerId = 0;

// Send message to the logServer every 10 seconds
Task myLoggingTask(10000, TASK_FOREVER, []()
                   {
                     DynamicJsonDocument jsonBuffer(1024);
                     JsonObject msg = jsonBuffer.to<JsonObject>();

                     msg["topic"] = "sensor";
                     msg["value"] = random(0, 180);

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

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6);
  mesh.onReceive(&receivedCallback);

  // Add the task to the your scheduler
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
