#include <SPI.h>
#include "epd2in13b.h"
#include "epdpaint.h"

#define COLORED   0
#define UNCOLORED 1

// Panel: Holitech HINK-E0213A07, 2.13" B/W/R, 212x104, SSD1675B
// SPI pins use the ESP8266 hardware SPI. Control pins are defined in epdif.h
// BUSY=5, RST=2, DC=4, CS=15 (edit epdif.h if your wiring differs)

void setup() {
  Serial.begin(115200);
  delay(50);

  Epd epd;
  if (epd.Init() != 0) {
    Serial.println("EPD init failed");
    return;
  }

  // Clear the panel SRAM
  epd.ClearFrame();

  // A small drawing buffer (width must be multiple of 8). 104 is panel width.
  // Using 104x64 here => (104/8)*64 = 13*64 = 832 bytes
  static unsigned char image[13 * 64];
  Paint paint(image, 104, 32);

  // Header (black)
  paint.Clear(UNCOLORED);
  paint.DrawStringAt(2, 2, "HINK-E0213A07", &Font12, COLORED);
  epd.SetPartialWindowBlack(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());

  // Subheader (red)
  paint.Clear(COLORED);
  paint.DrawStringAt(2, 2, "212x104 SSD1675B", &Font12, UNCOLORED);
  epd.SetPartialWindowRed(paint.GetImage(), 0, 32, paint.GetWidth(), paint.GetHeight());

  // Black graphics block
  paint.SetWidth(104);
  paint.SetHeight(64);
  paint.Clear(UNCOLORED);
  paint.DrawRectangle(0, 0, 103, 63, COLORED);
  paint.DrawLine(0, 0, 103, 63, COLORED);
  paint.DrawLine(103, 0, 0, 63, COLORED);
  epd.SetPartialWindowBlack(paint.GetImage(), 0, 68, 104, 64);

  // Red graphics block
  paint.Clear(UNCOLORED);
  paint.DrawFilledCircle(20, 20, 18, COLORED);
  paint.DrawFilledRectangle(50, 4, 100, 28, COLORED);
  epd.SetPartialWindowRed(paint.GetImage(), 0, 136, 104, 64);

  // Update display
  epd.DisplayFrame();

  // Optional: deep sleep to save power
  epd.Sleep();
}

void loop() {
  // Nothing to do
}

