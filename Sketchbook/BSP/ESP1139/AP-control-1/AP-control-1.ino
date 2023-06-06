#define onModule_LED 2
#define relay1 12
#define relay2 13
#define onBoard_LED 16

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#ifndef APSSID
#define APSSID "ESPap"
#define APPSK  "thereisnospoon"
#endif

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

ESP8266WebServer server(80);

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
void handleRoot() {
  server.send(200, "text/html", "<h1>You are connected</h1>");
}

void m1() {
  server.send(200, "text/html", "<h1>onModule LED ON</h1>");
  digitalWrite(onModule_LED, LOW);
}

void m0() {
  server.send(200, "text/html", "<h1>onModule LED OFF</h1>");
  digitalWrite(onModule_LED, HIGH);
}

void r11() {
  server.send(200, "text/html", "<h1>Relay1 ON</h1>");
  digitalWrite(relay1, LOW);
}

void r10() {
  server.send(200, "text/html", "<h1>Relay1 OFF</h1>");
  digitalWrite(relay1, HIGH);
}

void r21() {
  server.send(200, "text/html", "<h1>Relay2 ON</h1>");
  digitalWrite(relay2, LOW);
}

void r20() {
  server.send(200, "text/html", "<h1>Relay2 OFF</h1>");
  digitalWrite(relay2, HIGH);
}

void b1() {
  server.send(200, "text/html", "<h1>onBoard LED ON</h1>");
  digitalWrite(onBoard_LED, LOW);
}

void b0() {
  server.send(200, "text/html", "<h1>onBoard LED OFF</h1>");
  digitalWrite(onBoard_LED, HIGH);
}


void setup() {

  pinMode(onModule_LED, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(onBoard_LED, OUTPUT);

  blink_loop();
  
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.on("/", handleRoot);
  server.on("/m1", m1);
  server.on("/m0", m0);
  server.on("/r11", r11);
  server.on("/r10", r10);
  server.on("/r21", r21);
  server.on("/r20", r20);
  server.on("/b1", b1);
  server.on("/b0", b0);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}


void blink_loop() {
  blink_all();
  blink_all();
  blink_all();
}

void blink_all() {
  digitalWrite(onModule_LED, LOW);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(onBoard_LED, HIGH);
  delay(2000);
  digitalWrite(onModule_LED, HIGH);
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(onBoard_LED, LOW);
  delay(2000);

}
