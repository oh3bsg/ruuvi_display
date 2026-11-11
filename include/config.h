#pragma once

// MAC address of the Ruuvi tag to show (case-insensitive, colon separated).
// #define RUUVI_TAG_MAC "DC:9C:6B:B4:D6:27"  // Sisätila
#define RUUVI_TAG_MAC "F1:8C:AA:9B:30:14"  // Ulkoanturi


// Pressure trend arrow: one sample every 10 min, 7 samples (= 60 min window).
// Arrow shows rising/falling when the change over the window reaches the threshold.
#define PRESSURE_SAMPLE_INTERVAL_MS   (10UL * 60UL * 1000UL)
#define PRESSURE_HISTORY_SIZE         7
#define PRESSURE_TREND_THRESHOLD_HPA  0.5f

// Temperature colour: cold below TEMP_COLD_BELOW_C, hot above TEMP_HOT_ABOVE_C,
// white in between. Colours are RGB565 (0x001F = blue, 0xF800 = red).
#define TEMP_COLD_BELOW_C  0.0f
#define TEMP_HOT_ABOVE_C   25.0f
#define TEMP_COLD_COLOR    0x041F  // lighter blue, easier to read on black
#define TEMP_HOT_COLOR     0xF800
