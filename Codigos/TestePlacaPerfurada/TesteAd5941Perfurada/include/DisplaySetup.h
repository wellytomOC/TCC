#pragma once


// ==========================================================
//  Custom TFT_eSPI setup for ILI9488 + LVGL
// ==========================================================

// --- Display driver and size ---
#define ILI9488_DRIVER
#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// --- Pin definitions ---
#define TFT_MISO 5
#define TFT_MOSI 4
#define TFT_SCLK 6
#define TFT_CS   1
#define TFT_DC   3
#define TFT_RST  2

// --- Touch (optional, only if using XPT2046 or similar) ---
#define TOUCH_CS  7
#define TOUCH_IRQ 8

// --- Features ---
#define LOAD_GLCD
#define USE_HSPI_PORT
#define SPI_FREQUENCY 40000000  // 40 MHz (try lowering if unstable)
#define SPI_READ_FREQUENCY 20000000


    