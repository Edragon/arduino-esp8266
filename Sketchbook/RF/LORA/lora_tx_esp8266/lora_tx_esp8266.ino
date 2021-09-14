

#include <SPI.h>
#include <RH_RF95.h>


#define RFM95_CS 15
#define RFM95_RST 16
#define RFM95_INT 4

#define RF95_FREQ 434.0

#define LED 2


RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton instance of the radio driver
//RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() 
{

  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);   
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(115200);
  delay(100);

  Serial.println();

  Serial.println("Gateway Module starting…");

  // reset lora module
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {

    Serial.println("LoRa radio init failed");

    while (1);

  }

  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {

    Serial.println("setFrequency failed");

    while (1);

  }

  Serial.print("Set Freq to: ");  
  Serial.println(RF95_FREQ);  
  rf95.setTxPower(23, false);

}


int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  char radiopacket[32] = "Electrodragon - #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[31] = 0;
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);
  
  digitalWrite(LED, HIGH);

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  
  delay(1000);
    digitalWrite(LED, LOW);
}
