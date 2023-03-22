 
/*
 * Sketch: ESP8266_Part8_01_AutoUpdate_HTML
 * Intended to be run on an ESP8266
 */
 
String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
 
String html_1 = R"=====(
<!DOCTYPE html>
<html>


 <head>
 
 <meta name='viewport' content='width=device-width, initial-scale=1.0'/>
 <meta charset='utf-8'>
 <meta http-equiv='refresh' content='5'>

 
 <style>
   body {font-size:100%;} 
   #main {display: table; margin: auto;  padding: 0 10px 0 10px; } 
   h2 {text-align:center; } 
   p { text-align:center; }
 </style>

 
   <title>Test ...</title>
 </head>

 
 <body>
   <div id='main'>
     <h2>Auto Update Example Using HTML Electrodragon</h2>
     
     <div id='count'> 
       <p>Count = %count%</p>
     </div>
     
   </div> 
   
 </body>
</html>
)====="; 


 
#include <ESP8266WiFi.h>
 
// change these values to match your network
char ssid[] = "111";       //  your network SSID (name)
char pass[] = "electrodragon";          //  your network password
 
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
