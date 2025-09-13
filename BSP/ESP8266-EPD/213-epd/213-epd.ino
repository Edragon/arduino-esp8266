/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_EPD.h"

// ESP8266 pin mapping
#define EPD_DC 4
#define EPD_CS 15
#define EPD_BUSY 5 // set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS -1 // use internal RAM (no external SRAM connected)
#define EPD_RESET 2  // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI // primary SPI

// 2.13" b/w EPD (250x122) with SSD1675B controller
Adafruit_SSD1675B display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

#define COLOR1 EPD_BLACK
#define COLOR2 EPD_BLACK // monochrome panel; use black for both passes

// Debug helpers: print free heap on ESP8266
#define PRINT_FREE_HEAP()                                  \
  do {                                                     \
    Serial.print("[DBG] Free heap: ");                    \
    Serial.print(ESP.getFreeHeap());                       \
    Serial.println(" bytes");                             \
  } while (0)

static void printPinConfig() {
  Serial.println("[DBG] Pin configuration:");
  Serial.print("  EPD_DC="); Serial.println(EPD_DC);
  Serial.print("  EPD_CS="); Serial.println(EPD_CS);
  Serial.print("  EPD_BUSY="); Serial.println(EPD_BUSY);
  Serial.print("  EPD_RESET="); Serial.println(EPD_RESET);
  Serial.print("  SRAM_CS="); Serial.println(SRAM_CS);
  Serial.println("  EPD_SPI = SPI (primary SPI)");
#if (SRAM_CS == -1)
  Serial.println("[DBG] Using internal MCU RAM for frame buffer (SRAM_CS = -1)");
#else
  Serial.println("[DBG] Using external SRAM for frame buffer");
#endif
}

static void printBusyState(const char *tag) {
#if (EPD_BUSY != -1)
  pinMode(EPD_BUSY, INPUT);
  Serial.print("[DBG] BUSY ("); Serial.print(tag); Serial.print(") = ");
  Serial.println(digitalRead(EPD_BUSY));
#else
  Serial.print("[DBG] BUSY ("); Serial.print(tag); Serial.println(") unused (-1)");
#endif
}

static void fullScreenFill(uint16_t color, const char *label) {
  Serial.print("[DBG] Full-screen fill: "); Serial.println(label);
  display.clearBuffer();
  display.fillScreen(color);
  uint32_t t0 = millis();
  display.display();
  uint32_t t1 = millis();
  Serial.print("[DBG] Fill '" ); Serial.print(label); Serial.print("' update took "); Serial.print(t1 - t0); Serial.println(" ms");
  PRINT_FREE_HEAP();
}

static void runEPDDiagnostic() {
  Serial.println("[DBG] ===== EPD Diagnostic: start =====");
  // Variant A: library defaults
  Serial.println("[DBG] Variant A: default buffer/inversion");
  fullScreenFill(EPD_BLACK, "BLACK (default)");
  delay(1500);
  fullScreenFill(EPD_WHITE, "WHITE (default)");
  delay(800);

  // Variant B: try black buffer index 0 inverted
  Serial.println("[DBG] Variant B: setBlackBuffer(0, true)");
  display.setBlackBuffer(0, true);
  fullScreenFill(EPD_BLACK, "BLACK (idx0,invert)");
  delay(1500);
  fullScreenFill(EPD_WHITE, "WHITE (idx0,invert)");
  delay(800);

  // Variant C: try black buffer index 1 inverted
  Serial.println("[DBG] Variant C: setBlackBuffer(1, true)");
  display.setBlackBuffer(1, true);
  fullScreenFill(EPD_BLACK, "BLACK (idx1,invert)");
  delay(1500);
  fullScreenFill(EPD_WHITE, "WHITE (idx1,invert)");
  delay(800);

  Serial.println("[DBG] ===== EPD Diagnostic: end =====");
}

void setup() {
  Serial.begin(115200);
  // while (!Serial) { delay(10); }
  Serial.println("Adafruit EPD test");
  Serial.println("[DBG] Booting EPD test...");
  printPinConfig();
  PRINT_FREE_HEAP();

  SPI.begin();
  SPI.setFrequency(4000000); // lower SPI clock for signal integrity
  Serial.println("[DBG] SPI.begin + SPI @ 4MHz");

  printBusyState("before begin");
  Serial.println("[DBG] Calling display.begin()...");
  uint32_t t0 = millis();
  display.begin();
  uint32_t t1 = millis();
  printBusyState("after begin");
  Serial.print("[DBG] display.begin() took "); Serial.print(t1 - t0); Serial.println(" ms");
  Serial.print("[DBG] Display size: "); Serial.print(display.width()); Serial.print(" x "); Serial.println(display.height());
  display.setRotation(0);
  Serial.print("[DBG] Rotation set to "); Serial.println(0);

  // Quick diagnostic to verify polarity/driver. Comment out if not needed.
  runEPDDiagnostic();

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
  printBusyState("before text update");
  display.display();
  t1 = millis();
  printBusyState("after text update");
  Serial.print("[DBG] Display update took "); Serial.print(t1 - t0); Serial.println(" ms");
  PRINT_FREE_HEAP();

  delay(5000);

  Serial.println("[DBG] Clearing buffer for line test...");
  display.clearBuffer();
  Serial.println("[DBG] Drawing diagonal fan lines (BLACK)...");
  for (int16_t i = 0; i < display.width(); i += 4) {
    display.drawLine(0, 0, i, display.height() - 1, COLOR1);
    // yield to avoid WDT resets
    if ((i & 0x3F) == 0) yield();
  }

  Serial.println("[DBG] Drawing diagonal fan lines (COLOR2)...");
  for (int16_t i = 0; i < display.height(); i += 4) {
    display.drawLine(display.width() - 1, 0, 0, i, COLOR2);
    if ((i & 0x3F) == 0) yield();
  }
  Serial.println("[DBG] Updating display (lines)...");
  t0 = millis();
  printBusyState("before line update");
  display.display();
  t1 = millis();
  printBusyState("after line update");
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
