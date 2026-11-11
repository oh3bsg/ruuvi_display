#pragma once
// Wiring: Seeed XIAO nRF52840 <-> ILI9341 TFT (SPI, 3.3 V)
//
//  ILI9341        XIAO nRF52840
//  VCC       ->   3V3
//  GND       ->   GND
//  CS        ->   D1
//  RESET     ->   D2
//  DC/RS     ->   D3
//  SDI/MOSI  ->   D10 (hardware SPI MOSI)
//  SCK       ->   D8  (hardware SPI SCK)
//  LED       ->   3V3 (or D4 for PWM backlight dimming)
//  SDO/MISO  ->   D9  (hardware SPI MISO, optional)

#define TFT_PIN_CS   D1
#define TFT_PIN_RST  D2
#define TFT_PIN_DC   D3
#define TFT_PIN_BL   D4   // optional, only if backlight is driven from a GPIO
#define TFT_PIN_SCK  D8
#define TFT_PIN_MISO D9
#define TFT_PIN_MOSI D10
