/*
  WebSerial Demo
  ------
  This example code works for both ESP8266 & ESP32 Microcontrollers
  WebSerial is accessible at your ESP's <IPAddress>/webserial URL.

  Author: Ayush Sharma
  Checkout WebSerial Pro: https://webserial.pro
*/
#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <AsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include <SoftwareSerial.h>
SoftwareSerial swSer;


AsyncWebServer server(80);

const char* ssid = "111"; // Your WiFi SSID
const char* password = "electrodragon"; // Your WiFi Password


/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len) {
  WebSerial.println("Received Data...");
  String d = "";
  for (int i = 0; i < len; i++) {
    d += char(data[i]);
  }
  WebSerial.println(d);
}

void setup() {
  Serial.begin(4800);
  swSer.begin(4800, SWSERIAL_8N1, 4, 5, false, 95, 11);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  /* Attach Message Callback */
  WebSerial.msgCallback(recvMsg);
  server.begin();
}

void loop() {
  int packet[50];
  int packetLength = 0;

  while ( Serial.available() ) {
    char ch = Serial.read();

    if (packetLength < 50 ) {    //finalise
      packet[packetLength++] = ch;
    } else {
      for (int i = 0; i < 50; i++) {

        WebSerial.print(packet[i]);
        WebSerial.print(" ");
        swSer.print(packet[i]);
        swSer.print(" ");
        packet[i] = 0;
      }
      WebSerial.println(" ");
      packetLength = 0;
      delay(2000);
    }
  }

}
