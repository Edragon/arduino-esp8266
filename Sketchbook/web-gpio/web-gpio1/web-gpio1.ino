#include <ESP8266WiFi.h>

const char* ssid = "111";
const char* password = "electrodragon";

int PCpin = 12; 

WiFiServer server(80);

void setup() 
{
  Serial.begin(115200);

  pinMode(PCpin, OUTPUT);
  digitalWrite(PCpin, LOW);
  
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.print("IP: ");  Serial.println(WiFi.localIP());

  server.begin();
}



void loop() 
{
  WiFiClient client = server.available();
  if (!client) {
  return;
  }

  while(!client.available()){
  }
  
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();

  int value = LOW;
  if (request.indexOf("/PC=ON") != -1) 
  {
    digitalWrite(PCpin, HIGH);
    value = HIGH;
  } 
  if (request.indexOf("/PC=OFF") != -1)
  {
    digitalWrite(PCpin, LOW);
    value = LOW;
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one

  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.print("LED status: "); 

  if(value == HIGH) 
  {
    client.print("ON");  
  } 
  else 
  {
    client.print("OFF");
  }
  
  client.println("<br><br>");
  client.println("Turn <a href=\"/PC=ON\">ON</a><br>");
  client.println("Turn <a href=\"/PC=OFF\">OFF</a><br>");
  client.println("</html>");

  Serial.println("");
}
