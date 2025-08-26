/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_EPD.h"

#ifdef ARDUINO_ADAFRUIT_FEATHER_RP2040_THINKINK // detects if compiling for
                                                // Feather RP2040 ThinkInk
#define EPD_DC PIN_EPD_DC       // ThinkInk 24-pin connector DC
#define EPD_CS PIN_EPD_CS       // ThinkInk 24-pin connector CS
#define EPD_BUSY PIN_EPD_BUSY   // ThinkInk 24-pin connector Busy
#define SRAM_CS -1              // use onboard RAM
#define EPD_RESET PIN_EPD_RESET // ThinkInk 24-pin connector Reset
#define EPD_SPI &SPI1           // secondary SPI for ThinkInk
#else
#define EPD_DC 4
#define EPD_CS 15
#define EPD_BUSY 5 // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS -1
#define EPD_RESET 2  // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI // primary SPI
#endif

// Uncomment the following line if you are using 1.54" EPD with IL0373
// Adafruit_IL0373 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with SSD1680
// Adafruit_SSD1680 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with SSD1608
// Adafruit_SSD1608 display(200, 200, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with SSD1681
// Adafruit_SSD1681 display(200, 200, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with UC8151D
// Adafruit_UC8151D display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with SSD1680
// Adafruit_SSD1680 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with SSD1675
// Adafruit_SSD1675 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with SSD1675B
Adafruit_SSD1675B display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with UC8151D
// Adafruit_UC8151D display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with IL0373
// Adafruit_IL0373 display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
//#define FLEXIBLE_213

// Uncomment the following line if you are using 2.7" EPD with IL91874
// Adafruit_IL91874 display(264, 176, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.7" EPD with EK79686
// Adafruit_EK79686 display(264, 176, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.9" EPD with IL0373
// Adafruit_IL0373 display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI); #define FLEXIBLE_290

// Uncomment the following line if you are using 2.9" EPD with SSD1680
// Adafruit_SSD1680 display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.9" EPD with UC8151D
// Adafruit_UC8151D display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

#define COLOR1 EPD_BLACK
#define COLOR2 EPD_RED

// Debug helpers: print free heap on ESP8266/ESP32
#if defined(ESP8266) || defined(ESP32)
#define PRINT_FREE_HEAP()                         \
  do {                                            \
    Serial.print("[DBG] Free heap: ");           \
    Serial.print(ESP.getFreeHeap());              \
    Serial.println(" bytes");                    \
  } while (0)
#else
#define PRINT_FREE_HEAP() do { } while (0)
#endif

static void printPinConfig() {
  Serial.println("[DBG] Pin configuration:");
  Serial.print("  EPD_DC="); Serial.println(EPD_DC);
  Serial.print("  EPD_CS="); Serial.println(EPD_CS);
  Serial.print("  EPD_BUSY="); Serial.println(EPD_BUSY);
  Serial.print("  EPD_RESET="); Serial.println(EPD_RESET);
  Serial.print("  SRAM_CS="); Serial.println(SRAM_CS);
#if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_THINKINK)
  Serial.println("  EPD_SPI = SPI1 (ThinkInk secondary SPI)");
#else
  Serial.println("  EPD_SPI = SPI (primary SPI)");
#endif
#if (SRAM_CS == -1)
  Serial.println("[DBG] Using internal MCU RAM for frame buffer (SRAM_CS = -1)");
#else
  Serial.println("[DBG] Using external SRAM for frame buffer");
#endif
}

void setup() {
  Serial.begin(115200);
  // while (!Serial) { delay(10); }
  Serial.println("Adafruit EPD test");
  Serial.println("[DBG] Booting EPD test...");
  printPinConfig();
  PRINT_FREE_HEAP();

  Serial.println("[DBG] Calling display.begin()...");
  uint32_t t0 = millis();
  display.begin();
  uint32_t t1 = millis();
  Serial.print("[DBG] display.begin() took "); Serial.print(t1 - t0); Serial.println(" ms");
  Serial.print("[DBG] Display size: "); Serial.print(display.width()); Serial.print(" x "); Serial.println(display.height());

#if defined(FLEXIBLE_213) || defined(FLEXIBLE_290)
  // The flexible displays have different buffers and invert settings!
  Serial.println("[DBG] Applying FLEXIBLE display buffer settings");
  display.setBlackBuffer(1, false);
  display.setColorBuffer(1, false);
#endif

  // large block of text
  Serial.println("[DBG] Clearing buffer for text...");
  display.clearBuffer();
  const char *msg =
      "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur "
      "adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, "
      "fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor "
      "neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet "
      "ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a "
      "tortor imperdiet posuere. ";
  Serial.print("[DBG] Drawing text, length="); Serial.println(strlen(msg));
  testdrawtext(msg, COLOR1);
  Serial.println("[DBG] Updating display (text)...");
  t0 = millis();
  display.display();
  t1 = millis();
  Serial.print("[DBG] Display update took "); Serial.print(t1 - t0); Serial.println(" ms");
  PRINT_FREE_HEAP();

  delay(5000);

  Serial.println("[DBG] Clearing buffer for line test...");
  display.clearBuffer();
  Serial.println("[DBG] Drawing diagonal fan lines (BLACK)...");
  for (int16_t i = 0; i < display.width(); i += 4) {
    display.drawLine(0, 0, i, display.height() - 1, COLOR1);
    // yield on ESP to avoid WDT
#if defined(ESP8266)
    if ((i & 0x3F) == 0) yield();
#endif
  }

  Serial.println("[DBG] Drawing diagonal fan lines (COLOR2)...");
  for (int16_t i = 0; i < display.height(); i += 4) {
    display.drawLine(display.width() - 1, 0, 0, i, COLOR2); // on grayscale this will be mid-gray
#if defined(ESP8266)
    if ((i & 0x3F) == 0) yield();
#endif
  }
  Serial.println("[DBG] Updating display (lines)...");
  t0 = millis();
  display.display();
  t1 = millis();
  Serial.print("[DBG] Display update took "); Serial.print(t1 - t0); Serial.println(" ms");
  PRINT_FREE_HEAP();
}

void loop() {
  // don't do anything!
}

void testdrawtext(const char *text, uint16_t color) {
  Serial.print("[DBG] testdrawtext color="); Serial.println(color);
  display.setCursor(0, 0);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}
