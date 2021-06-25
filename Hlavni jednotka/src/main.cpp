#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <ArduinoJson.h>

#include <OneWire.h>
#include <DallasTemperature.h>
const int oneWireBus = 0;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
// Variables to store temperature values
String temperatureC = "";

String readDSTemperatureC() {
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 
  float tempC = sensors.getTempCByIndex(0);

  if(tempC == -127.00) {
    Serial.println("Failed to read from DS18B20 sensor");
    return "--";
  } else {
    Serial.print("Temperature Celsius: ");
    Serial.println(tempC); 
  }
  return String(tempC);
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <style>
    html {
     font-family: Arial;
     display: inline-block;
     margin: 0px auto;
     text-align: center;
    }
    h2 { font-size: 3.0rem; }
    p { font-size: 3.0rem; }
    .units { font-size: 1.2rem; }
    .ds-labels{
      font-size: 1.5rem;
      vertical-align:middle;
      padding-bottom: 15px;
    }
  </style>
</head>
<body>
  <h2>ESP DS18B20 Server</h2>
  <p>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="ds-labels">Temperature Celsius</span> 
    <span id="temperaturec">%TEMPERATUREC%</span>
    <sup class="units">&deg;C</sup>
  </p>
</body>
<script>
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("temperaturec").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/temperaturec", true);
  xhttp.send();
}, 10000) ;

</script>
</html>)rawliteral";

// Replaces placeholder with DS18B20 values
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATUREC"){
    return temperatureC;
  }
  return String();
}

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

   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/temperaturec", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", temperatureC.c_str());
  });
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
  temperatureC = readDSTemperatureC();
  AsyncElegantOTA.loop(); // OTA
  PrijemDat();
  WiFi_reconnect();
}