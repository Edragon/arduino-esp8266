#include <ESP8266WiFi.h>
#include "test.h"

WiFiServer server(80);
 
String tmpString = "";

//unsigned int count = 0; 


 
void setup() 
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("Serial started at 115200");
    Serial.println();
 
    // Connect to a WiFi network
    Serial.print(F("Connecting to "));  Serial.println(ssid);
    WiFi.begin(ssid, pass);
 
    while (WiFi.status() != WL_CONNECTED) 
    {
        Serial.print(".");
        delay(500);
    }
 
    Serial.println("");
    Serial.println(F("[CONNECTED]"));
    Serial.print("[IP ");              
    Serial.print(WiFi.localIP()); 
    Serial.println("]");
 
    // start a server
    server.begin();
    Serial.println("Server started");
 
} 

 
void loop() 
{
    // Check if a client has connected
    WiFiClient client = server.available();

    int sensorValue = analogRead(A0);
    //int outputValue = map(sensorValue, 0, 1023, 0, 255);
    
    //if (!client)  {  return;  }
 
    //count ++;
 
    tmpString = html_1;
    tmpString.replace("%count%", String(sensorValue) );
 
    client.flush();
    client.print( header );
    client.print( tmpString );  
 
    Serial.print("count = "); Serial.println(sensorValue); 
    delay(5);

} 
