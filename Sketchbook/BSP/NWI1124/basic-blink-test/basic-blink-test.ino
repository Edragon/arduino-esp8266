

#define OM_LED 2 // on module led
#define WS_LED 2 // WS2812
#define OB_LED 16 // on board led

#define RF_LK 0 // wireless control, signal to LOW, default and boot HIGH

#define B_LED 12 // channel B Blue
#define G_LED 13 // channel G Blue
#define W_LED 14 // channel W Blue
#define R_LED 15 // channel R Blue

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <uri/UriBraces.h>
#include <uri/UriRegex.h>

#ifndef STASSID
#define STASSID "111"
#define STAPSK  "electrodragon"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

ESP8266WebServer server(80);

void setup(void) {

  pinMode(OM_LED, OUTPUT);  // on module led
  pinMode(OB_LED, OUTPUT); // on board led

  pinMode(12, OUTPUT); // channel B Blue
  pinMode(13, OUTPUT); // channel G Green
  pinMode(14, OUTPUT); // channel W white
  pinMode(15, OUTPUT); // channel R Red

  digitalWrite(OM_LED, HIGH);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

}

void loop(void) {
    test_repeat();
}



void test_repeat() {
  Serial.println("Testing LEDs .. ");
  test_LED ();
  test_LED ();

  test_LED ();
  test_LED ();

  test_LED ();
  test_LED ();

  test_LED ();
  test_LED ();

  test_LED ();
  test_LED ();
}


void test_LED () {
  Serial.println("test ..");
  digitalWrite(OB_LED, LOW);
  digitalWrite (B_LED, LOW);
  digitalWrite (G_LED, LOW);
  digitalWrite (W_LED, LOW);
  digitalWrite (R_LED, LOW);
  delay(1000);

  digitalWrite(OB_LED, HIGH);
  digitalWrite (B_LED, HIGH);
  digitalWrite (G_LED, HIGH);
  digitalWrite (W_LED, HIGH);
  digitalWrite (R_LED, HIGH);
  delay(10000);

}
