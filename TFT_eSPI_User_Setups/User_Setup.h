/*
C:\Users\<user>\Documents\Arduino\libraries\TFT_eSPI\User_Setup.h
*/

#define ST7735_DRIVER
#define TFT_RGB_ORDER TFT_RGB
#define TFT_WIDTH  128
#define TFT_HEIGHT 160
#define ST7735_BLACKTAB
#define TFT_INVERSION_OFF

// Config for Herculito

#define TFT_MISO  12 // default 19, -1 does not work, blocks 19 instead
#define TFT_MOSI  21
#define TFT_SCLK  22
#define TFT_CS    15  // Chip select control pin
#define TFT_DC    33  // Data Command control pin
#define TFT_RST   -1  // default 4, -1 does not work, blocks 4 instead

/*
// config for ESP32 demo
#define TFT_MISO  19  // not broken out on this board. 
#define TFT_MOSI  23
#define TFT_SCLK  18
#define TFT_CS    15  // Chip select control pin
#define TFT_DC     2  // Data Command control pin
#define TFT_RST   -1  // Reset pin (could connect to RST pin) // 4
*/

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters

#define SPI_FREQUENCY  27000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

