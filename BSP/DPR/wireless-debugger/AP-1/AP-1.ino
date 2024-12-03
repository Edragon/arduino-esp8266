
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>


// Set these to your desired credentials.
const char *ssid = "ESP0000";
const char *password = "1234567890";

WiFiServer server(8080);


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);//灭
  
  Serial.begin(500000);
  Serial.setTimeout(1);

  Serial.println ("setup start ");

  //WiFi.mode(WIFI_AP);
  //WiFi.setSleepMode(WIFI_NONE_SLEEP);
  // You can remove the password parameter if you want the AP to be open.
  //  WiFi.softAPdisconnect(true);
  //WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  //  IPAddress myIP = WiFi.softAPIP();
  server.begin();
  Serial.println ("all setup done ");
}

void loop()
{
  WiFiClient client = server.available();   // listen for incoming clients
  digitalWrite(LED_BUILTIN, 1);//灭
  //  delay(2000);
  if (client)
  {
    delay(100);
    digitalWrite(LED_BUILTIN, 0);//亮
    client.setNoDelay(true);
    while (1)
    {
      if (client.available()) //如果有数据可读取
      {
        char line = client.read(); //读取数据到换行符
        Serial.print(line);
      }
      if (Serial.available()) //如果有数据可读取
      {
        String s = Serial.readString();
        client.print(s);
      }
      if (WiFi.softAPgetStationNum() == 0)
      {
        //          WiFi.softAPdisconnect(true);
        //          WiFi.softAPConfig(local_IP,gateway,subnet);
        //          WiFi.softAP(ssid, password);
        //          server.begin();
        break;
      }
    }
  }


  //  WiFi.softAPdisconnect(true);
  //  WiFi.softAPConfig(local_IP,gateway,subnet);
  //  WiFi.softAP(ssid, password);
  //  server.begin();
}
