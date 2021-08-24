#include "painlessMesh.h"
#include "credentials.h"

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
// Prototype
void receivedCallback(uint32_t from, String &msg);

// Send my ID every 10 seconds to inform others
Task logServerTask(10000, TASK_FOREVER, []()
                   {
                     DynamicJsonDocument jsonBuffer(1024);
                     JsonObject msg = jsonBuffer.to<JsonObject>();

                     msg["topic"] = "logServer";
                     msg["nodeId"] = mesh.getNodeId();

                     String str;

                     serializeJson(msg, str);

                     mesh.sendBroadcast(str);

                     // log to serial
                     /* serializeJson(msg, Serial);

                    Serial.printf("\n");
                    */
                   });

void setup()
{
  Serial.begin(57600);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE | DEBUG ); // all types on
  //mesh.setDebugMsgTypes( ERROR | CONNECTION | SYNC | S_TIME );  // set before init() so that you can see startup messages
  mesh.setDebugMsgTypes(ERROR | CONNECTION | S_TIME); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6);
  mesh.onReceive(&receivedCallback);

  mesh.onNewConnection([](size_t nodeId)
                       { Serial.printf("New Connection %u\n", nodeId); });

  mesh.onDroppedConnection([](size_t nodeId)
                           { Serial.printf("Dropped Connection %u\n", nodeId); });

  mesh.initOTAReceive("bridge");

  // Add the task to the your scheduler
  userScheduler.addTask(logServerTask);
  logServerTask.enable();
}

void loop()
{
  // it will run the user scheduler as well
  mesh.update();
}

void receivedCallback(uint32_t from, String &msg)
{
  Serial.printf("logServer: Received from %u msg=%s\n", from, msg.c_str());
}
