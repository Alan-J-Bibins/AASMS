// Wokwi Custom Chip: NEO-6M GPS Simulator
// Emits $GPRMC and $GPGGA NMEA sentences every 1 second over UART
// Simulates a balloon ascending from VIT Vellore coordinates

#include "wokwi-api.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Simulation state
typedef struct {
  uart_dev_t uart;
  uint32_t   tick;         // seconds elapsed
  timer_t    timer;
} chip_state_t;

static chip_state_t chip;

// XOR checksum for NMEA sentences
static uint8_t nmea_checksum(const char *sentence) {
  uint8_t cs = 0;
  // Skip leading '$', stop before '*'
  for (const char *p = sentence + 1; *p && *p != '*'; p++) {
    cs ^= (uint8_t)(*p);
  }
  return cs;
}

static void send_nmea(void *user_data) {
  chip_state_t *state = (chip_state_t *)user_data;
  state->tick++;

  uint32_t t = state->tick;

  // Simulated position — VIT Vellore base, slowly drifting
  double lat     = 12.9716 + t * 0.0001;
  double lon     = 79.1587 + t * 0.00005;
  double alt_m   = t * 2.0;   // ascend 2 m/s
  double speed_k = 2.5;       // knots
  double heading = 45.0;

  // Convert decimal degrees to NMEA ddmm.mmmm format
  int    lat_deg = (int)lat;
  double lat_min = (lat - lat_deg) * 60.0;
  int    lon_deg = (int)lon;
  double lon_min = (lon - lon_deg) * 60.0;

  // Time: HHMMSS.00
  uint32_t hh = (t / 3600) % 24;
  uint32_t mm = (t / 60)   % 60;
  uint32_t ss =  t          % 60;

  char sentence[128];
  char full[140];

  // --- $GPRMC ---
  snprintf(sentence, sizeof(sentence),
    "$GPRMC,%02lu%02lu%02lu.00,A,%02d%07.4f,N,%03d%07.4f,E,%.1f,%.1f,080325,,,A",
    (unsigned long)hh, (unsigned long)mm, (unsigned long)ss,
    lat_deg, lat_min,
    lon_deg, lon_min,
    speed_k, heading);

  uint8_t cs = nmea_checksum(sentence);
  snprintf(full, sizeof(full), "%s*%02X\r\n", sentence, cs);
  uart_write(chip.uart, (uint8_t *)full, strlen(full));

  // --- $GPGGA ---
  snprintf(sentence, sizeof(sentence),
    "$GPGGA,%02lu%02lu%02lu.00,%02d%07.4f,N,%03d%07.4f,E,1,08,1.0,%.1f,M,0.0,M,,",
    (unsigned long)hh, (unsigned long)mm, (unsigned long)ss,
    lat_deg, lat_min,
    lon_deg, lon_min,
    alt_m);

  cs = nmea_checksum(sentence);
  snprintf(full, sizeof(full), "%s*%02X\r\n", sentence, cs);
  uart_write(chip.uart, (uint8_t *)full, strlen(full));
}

void chip_init(void) {
  chip.tick = 0;

  // Open UART at 9600 baud (matches NEO-6M default)
  uart_config_t uart_cfg = {
    .tx        = pin_init("TX", OUTPUT),
    .rx        = pin_init("RX", INPUT),
    .baud_rate = 9600,
    .rx_data   = NULL,
    .write_done = NULL,
    .user_data = NULL,
  };
  chip.uart = uart_init(&uart_cfg);

  // Fire every 1000 ms
  timer_config_t timer_cfg = {
    .callback  = send_nmea,
    .user_data = &chip,
  };
  chip.timer = timer_init(&timer_cfg);
  timer_start(chip.timer, 1000000, true); // 1,000,000 µs = 1 s, repeating
}