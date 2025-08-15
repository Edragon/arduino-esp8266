// include the library
#include <RadioLib.h>


SX1262 loraSX1262 = new Module(4, 15, 16);

void setup() {

  Serial.begin(9600);

  Serial.print(F("RE-Plug in the board in 10 seconds  ... "));

  delay(10000);  

  Serial.print(F("[SX1262] Initializing ... "));

  float lora_sx1262_freq = 878.0;                                                                               // MHz
  float lora_sx1262_bw = 125.0;                                                                                 // kHz
  uint8_t lora_sx1262_sf = 7;                                                                                   // spreading factor
  uint8_t lora_sx1262_cr = 5;                                                                                   // coding rate (4/5..4/8 uses 1..4 in RadioLib examples)
  uint16_t lora_sx1262_syncWord = 0x3444;                                                                       // sync word (16-bit)
  int8_t lora_sx1262_power = 20;                                                                                // dBm
  uint16_t lora_sx1262_preambleLength = 12;                                                                     // symbols
  float lora_sx1262_tcxoVoltage = 0.0;                                                                          // 0 = don't use TCXO, else e.g. 1.6, 2.4
  bool lora_sx1262_useRegulatorLDO = false;                                                                     // set true to use LDO regulator where supported

  int state = loraSX1262.begin(lora_sx1262_freq,
                               lora_sx1262_bw,
                               lora_sx1262_sf,
                               lora_sx1262_cr,
                               lora_sx1262_syncWord,
                               lora_sx1262_power,
                               lora_sx1262_preambleLength,
                               lora_sx1262_tcxoVoltage,
                               lora_sx1262_useRegulatorLDO);

  // int state = loraSX1262.begin(915.0, 125.0, 7, 5, 0x3444, 20, 12, 0, false);

  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true)
      ;
  }

  // set carrier frequency to 433.5 MHz
  if (loraSX1262.setFrequency(433.5) == ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true)
      ;
  }

  // set bandwidth to 250 kHz
  if (loraSX1262.setBandwidth(250.0) == ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true)
      ;
  }

  // set spreading factor to 10
  if (loraSX1262.setSpreadingFactor(10) == ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true)
      ;
  }

  // set coding rate to 6
  if (loraSX1262.setCodingRate(6) == ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true)
      ;
  }

  // set LoRa sync word to 0x1234
  if (loraSX1262.setSyncWord(0x1234) != ERR_NONE) {
    Serial.println(F("Unable to set sync word!"));
    while (true)
      ;
  }

  // set output power to 10 dBm (accepted range is -17 - 22 dBm)
  if (loraSX1262.setOutputPower(10) == ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true)
      ;
  }

  // set over current protection limit to 80 mA (accepted range is 45 - 240 mA)
  // NOTE: set value to 0 to disable overcurrent protection
  if (loraSX1262.setCurrentLimit(80) == ERR_INVALID_CURRENT_LIMIT) {
    Serial.println(F("Selected current limit is invalid for this module!"));
    while (true)
      ;
  }

  // set LoRa preamble length to 15 symbols (accepted range is 0 - 65535)
  if (loraSX1262.setPreambleLength(15) == ERR_INVALID_PREAMBLE_LENGTH) {
    Serial.println(F("Selected preamble length is invalid for this module!"));
    while (true)
      ;
  }

  // disable CRC
  if (loraSX1262.setCRC(false) == ERR_INVALID_CRC_CONFIGURATION) {
    Serial.println(F("Selected CRC is invalid for this module!"));
    while (true)
      ;
  }

  // Some SX126x modules have TCXO (temperature compensated crystal
  // oscillator). To configure TCXO reference voltage,
  // the following method can be used.
  if (loraSX1262.setTCXO(2.4) == ERR_INVALID_TCXO_VOLTAGE) {
    Serial.println(F("Selected TCXO voltage is invalid for this module!"));
    while (true)
      ;
  }

  // Some SX126x modules use DIO2 as RF switch. To enable
  // this feature, the following method can be used.
  // NOTE: As long as DIO2 is configured to control RF switch,
  //       it can't be used as interrupt pin!
  if (loraSX1262.setDio2AsRfSwitch() != ERR_NONE) {
    Serial.println(F("Failed to set DIO2 as RF switch!"));
    while (true)
      ;
  }

  Serial.println(F("All settings succesfully changed!"));
}

void loop() {
  // nothing here
}
