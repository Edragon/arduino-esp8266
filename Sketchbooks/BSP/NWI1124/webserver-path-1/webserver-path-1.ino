#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <uri/UriBraces.h>
#include <uri/UriRegex.h>

#ifndef STASSID
#define STASSID "123"
#define STAPSK  "electrodragon"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

ESP8266WebServer server(80);

void setup(void) {

  pinMode(2, OUTPUT);  // on module led
  pinMode(16, OUTPUT); // on board led
  pinMode(12, OUTPUT); // channel B Blue
  pinMode(13, OUTPUT); // channel G Green 
  pinMode(14, OUTPUT); // channel W white 
  pinMode(15, OUTPUT); // channel R Red

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on(F("/"), []() {
    server.send(200, "text/plain", "NWI1124 LED Board!");
  });

  server.on(UriRegex("^\\/io\\/([0-9]+)\\/val\\/([0-9]+)$"), []() {
    String esp_io = server.pathArg(0);
    String esp_val = server.pathArg(1);

    Serial.println(esp_io[0]);
    Serial.println(esp_val[0]);
    
    Serial.println(esp_io.toInt());
    Serial.println(esp_val.toInt());
    
    int ctrl_io = esp_io.toInt();
    int ctrl_val = esp_val.toInt();

    // example http://192.168.8.103/io/16/val/1
    //         http://192.168.8.103/io/2/val/0
    // send feedback
    server.send(200, "text/plain", "io: '" + esp_io + "' and val: '" + esp_val + "'");

    digitalWrite(ctrl_io, ctrl_val);

  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
}
