
#include "painlessMesh.h"
#include "credentials.h"

#include <FS.h>
#include "SD.h"
#include "SPI.h"

#define OTA_PART_SIZE 1024 //How many bytes to send per OTA data packet

painlessMesh mesh;

void rebootEspWithReason(String reason);

void setup() 
  {
  Serial.begin(115200);
  delay(1000);
  //mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION | DEBUG);  // set before init() so that you can see startup messages
  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION | DEBUG);  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);

  // Bridge node, should (in most cases) be a root node. See [the
  // wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation)
  // for some background
  mesh.setRoot(true);
  // This node and all other nodes should ideally know the mesh contains a root,
  // so call this on all nodes
  mesh.setContainsRoot(true);

  delay(1000);


  if (!SD.begin(4)) 
    {
    rebootEspWithReason("Could not mount SD card, restarting");
    }

  File dir = SD.open("/");
  while (true) 
    {
    File entry = dir.openNextFile();
    if (!entry) 
      { //End of files
      rebootEspWithReason("Could not find valid firmware, please validate");
      }

    //This block of code parses the file name to make sure it is valid.
    //It will also get the role and hardware the firmware is targeted at.
    if (!entry.isDirectory()) 
      {
      TSTRING name = entry.name();
      if (name.length() > 1 && name.indexOf('_') != -1 &&
          name.indexOf('_') != name.lastIndexOf('_') &&
          name.indexOf('.') != -1) 
        {
        TSTRING firmware  = name.substring(1, name.indexOf('_'));
        TSTRING hardware  = name.substring(name.indexOf('_') + 1, name.lastIndexOf('_'));
        TSTRING role      = name.substring(name.lastIndexOf('_') + 1, name.indexOf('.'));
        TSTRING extension = name.substring(name.indexOf('.') + 1, name.length());
        if (firmware.equals("firmware") && (hardware.equals("ESP8266") || hardware.equals("ESP32")) && extension.equals("bin")) 
          {
          Serial.printf("OTA FIRMWARE FOUND (%s), NOW BROADCASTING\n", entry.name());

          //This is the important bit for OTA, up to now was just getting the file. 
          //If you are using some other way to upload firmware, possibly from 
          //mqtt or something, this is what needs to be changed.
          //This function could also be changed to support OTA of multiple files
          //at the same time, potentially through using the pkg.md5 as a key in
          //a map to determine which to send
          mesh.initOTASend(
              [&entry](painlessmesh::plugin::ota::DataRequest pkg, char* buffer) 
                {                
                Serial.printf("OTASend %d %u\n", pkg.partNo, pkg.from);

                //fill the buffer with the requested data packet from the node.
                entry.seek(OTA_PART_SIZE * pkg.partNo);
                entry.readBytes(buffer, OTA_PART_SIZE);
                
                //The buffer can hold OTA_PART_SIZE bytes, but the last packet may
                //not be that long. Return the actual size of the packet.
                return min((unsigned)OTA_PART_SIZE,entry.size() - (OTA_PART_SIZE * pkg.partNo));
                },
              OTA_PART_SIZE);

          //Calculate the MD5 hash of the firmware we are trying to send. This will be used
          //to validate the firmware as well as tell if a node needs this firmware.
          MD5Builder md5;
          md5.begin();
          md5.addStream(entry, entry.size());
          md5.calculate(); 

          //Make it known to the network that there is OTA firmware available.
          //This will send a message every minute for an hour letting nodes know
          //that firmware is available.
          //This returns a task that allows you to do things on disable or more,
          //like closing your files or whatever.
          mesh.offerOTA(role, hardware, md5.toString(),ceil(((float)entry.size()) / OTA_PART_SIZE), false);

          while (true) 
            {
            //This program will not reach loop() so we dont have to worry about
            //file scope.
            mesh.update();
            delay(1000);
            }
          }
        }
      }
    }
  }   // setup

void rebootEspWithReason(String reason) {
  Serial.println(reason);
  delay(1000);
  ESP.restart();
}

void loop(){};