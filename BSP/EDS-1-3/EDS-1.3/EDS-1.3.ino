
//   Diagnostic test for the displayed colour order
//
// Written by Bodmer 17/2/19 for the TFT_eSPI library:
// https://github.com/Bodmer/TFT_eSPI

/* 
 Different hardware manufacturers use different colour order
 configurations at the hardware level.  This may result in
 incorrect colours being displayed.

 Incorrectly displayed colours could also be the result of
 using the wrong display driver in the library setup file.

 Typically displays have a control register (MADCTL) that can
 be used to set the Red Green Blue (RGB) colour order to RGB
 or BRG so that red and blue are swapped on the display.

 This control register is also used to manage the display
 rotation and coordinate mirroring. The control register
 typically has 8 bits, for the ILI9341 these are:

 Bit Function
 7   Mirror Y coordinate (row address order)
 6   Mirror X coordinate (column address order)
 5   Row/column exchange (for rotation)
 4   Refresh direction (top to bottom or bottom to top in portrait orientation)
 3   RGB order (swaps red and blue)
 2   Refresh direction (top to bottom or bottom to top in landscape orientation)
 1   Not used
 0   Not used

 The control register bits can be written with this example command sequence:
 
    tft.writecommand(TFT_MADCTL);
    tft.writedata(0x48);          // Bits 6 and 3 set
    
 0x48 is the default value for ILI9341 (0xA8 for ESP32 M5STACK)
 in rotation 0 orientation.
 
 Another control register can be used to "invert" colours,
 this swaps black and white as well as other colours (e.g.
 green to magenta, red to cyan, blue to yellow).
 
 To invert colours insert this line after tft.init() or tft.begin():

    tft.invertDisplay( invert ); // Where invert is true or false

*/

#include <SPI.h>

#include <TFT_eSPI.h>       // Hardware-specific library

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

void setup(void) {
  Serial.begin(115200);
  Serial.println("TFT Display Test Starting...");
  
  tft.init();
  
  // Fix for display offset issues - try these one at a time:
  
  // Option 1: Set column and row offset (common fix for offset issues)
  tft.setRotation(0);  // Try different rotations: 0, 1, 2, 3
  
  // Option 2: Manual offset correction - uncomment if data appears shifted
  // tft.writecommand(0x2A); // Column Address Set
  // tft.writedata(0x00);
  // tft.writedata(0x00);    // Start column (try 0x02 if offset)
  // tft.writedata(0x00);
  // tft.writedata(0x7F);    // End column
  
  // tft.writecommand(0x2B); // Row Address Set  
  // tft.writedata(0x00);
  // tft.writedata(0x00);    // Start row (try 0x01 or 0x02 if offset)
  // tft.writedata(0x00);
  // tft.writedata(0x9F);    // End row
  
  // Option 3: Try different color orders if display shows mosaic pattern
  tft.writecommand(TFT_MADCTL);
  // tft.writedata(0x48);  // Default for ILI9341
  // tft.writedata(0x40);  // Try this if colors are swapped
  // tft.writedata(0x08);  // Try this for different color order
  tft.writedata(0xC8);  // Try this for different orientation
  
  Serial.println("Display initialized");
  Serial.print("Width: "); Serial.println(tft.width());
  Serial.print("Height: "); Serial.println(tft.height());

  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);

  // Set "cursor" at top left corner of display (0,0) and select font 4
  tft.setCursor(0, 4, 4);

  // Set the font colour to be white with a black background
  tft.setTextColor(TFT_WHITE);

  // We can now plot text on screen using the "print" class
  tft.println(" Initialised default\n");
  tft.println(" White text");
  
  tft.setTextColor(TFT_RED);
  tft.println(" Red text");
  
  tft.setTextColor(TFT_GREEN);
  tft.println(" Green text");
  
  tft.setTextColor(TFT_BLUE);
  tft.println(" Blue text");

  delay(5000);
  
  // Test pattern to identify display offset issues
  Serial.println("Drawing offset diagnostic pattern...");
  tft.fillScreen(TFT_BLACK);
  
  // Draw a grid pattern to visualize offset
  for(int x = 0; x < tft.width(); x += 20) {
    tft.drawFastVLine(x, 0, tft.height(), TFT_WHITE);
  }
  for(int y = 0; y < tft.height(); y += 20) {
    tft.drawFastHLine(0, y, tft.width(), TFT_WHITE);
  }
  
  // Draw corner markers
  tft.fillRect(0, 0, 10, 10, TFT_RED);           // Top-left
  tft.fillRect(tft.width()-10, 0, 10, 10, TFT_GREEN);     // Top-right
  tft.fillRect(0, tft.height()-10, 10, 10, TFT_BLUE);     // Bottom-left
  tft.fillRect(tft.width()-10, tft.height()-10, 10, 10, TFT_YELLOW); // Bottom-right
  
  // Draw center cross
  int centerX = tft.width() / 2;
  int centerY = tft.height() / 2;
  tft.drawFastHLine(centerX - 20, centerY, 40, TFT_MAGENTA);
  tft.drawFastVLine(centerX, centerY - 20, 40, TFT_MAGENTA);
  
  delay(3000);
  
  // Additional diagnostic test - draw color bars
  Serial.println("Drawing color test bars...");
  tft.fillScreen(TFT_BLACK);
  
  int barHeight = tft.height() / 8;
  tft.fillRect(0, 0 * barHeight, tft.width(), barHeight, TFT_RED);
  tft.fillRect(0, 1 * barHeight, tft.width(), barHeight, TFT_GREEN);
  tft.fillRect(0, 2 * barHeight, tft.width(), barHeight, TFT_BLUE);
  tft.fillRect(0, 3 * barHeight, tft.width(), barHeight, TFT_YELLOW);
  tft.fillRect(0, 4 * barHeight, tft.width(), barHeight, TFT_MAGENTA);
  tft.fillRect(0, 5 * barHeight, tft.width(), barHeight, TFT_CYAN);
  tft.fillRect(0, 6 * barHeight, tft.width(), barHeight, TFT_WHITE);
  tft.fillRect(0, 7 * barHeight, tft.width(), barHeight, TFT_BLACK);
  
  delay(3000);

}

void loop() {
  
  Serial.println("=== Testing Normal Display ===");
  tft.invertDisplay( false ); // Where i is true or false
 
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);

  tft.setCursor(0, 4, 4);

  tft.setTextColor(TFT_WHITE);
  tft.println(" Invert OFF\n");

  tft.println(" White text");
  
  tft.setTextColor(TFT_RED);
  tft.println(" Red text");
  
  tft.setTextColor(TFT_GREEN);
  tft.println(" Green text");
  
  tft.setTextColor(TFT_BLUE);
  tft.println(" Blue text");

  delay(5000);

  Serial.println("=== Testing Inverted Display ===");
  // Binary inversion of colours
  tft.invertDisplay( true ); // Where i is true or false
 
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_GREEN);

  tft.setCursor(0, 4, 4);

  tft.setTextColor(TFT_WHITE);
  tft.println(" Invert ON\n");

  tft.println(" White text");
  
  tft.setTextColor(TFT_RED);
  tft.println(" Red text");
  
  tft.setTextColor(TFT_GREEN);
  tft.println(" Green text");
  
  tft.setTextColor(TFT_BLUE);
  tft.println(" Blue text");

  delay(5000);
}
