// Wokwi Custom Chip: SX1278 LoRa Simulator
// Handles SPI register reads/writes so the Arduino LoRa library
// initializes cleanly without hanging.
// Key behaviour:
//   - RegVersion (0x42) returns 0x12  → library confirms chip present
//   - DIO0 pulses HIGH after TX to signal TxDone interrupt
//   - All writes are acknowledged silently

#include "wokwi-api.h"
#include <stdio.h>
#include <string.h>

#define REG_FIFO        0x00
#define REG_OP_MODE     0x01
#define REG_VERSION     0x42
#define REG_PA_CONFIG   0x09
#define REG_IRQ_FLAGS   0x12
#define MODE_TX         0x83   // LoRa + TX mode

typedef struct {
  pin_t     pin_dio0;
  pin_t     pin_rst;
  pin_t     pin_nss;
  spi_dev_t spi;
  timer_t   tx_done_timer;

  uint8_t   regs[256];
  uint8_t   current_reg;
  bool      is_write;
  bool      first_byte;
  bool      tx_pending;
} chip_state_t;

static chip_state_t chip;

// Pulse DIO0 HIGH for 10 ms to signal TxDone
static void pulse_dio0(void *user_data) {
  chip_state_t *state = (chip_state_t *)user_data;
  pin_write(state->pin_dio0, HIGH);
  // Reset IRQ flags register so library can clear it
  state->regs[REG_IRQ_FLAGS] = 0x08; // TxDone bit
  state->tx_pending = false;
}

static uint8_t spi_rx(uint8_t data, void *user_data) {
  chip_state_t *state = (chip_state_t *)user_data;

  if (state->first_byte) {
    state->first_byte  = false;
    state->is_write    = (data & 0x80) != 0;
    state->current_reg = data & 0x7F;
    return 0x00;
  }

  if (state->is_write) {
    state->regs[state->current_reg] = data;

    // Detect TX mode being set — schedule DIO0 pulse in 500 ms
    if (state->current_reg == REG_OP_MODE &&
        data == MODE_TX &&
        !state->tx_pending) {
      state->tx_pending = true;
      timer_start(state->tx_done_timer, 500000, false); // 500 ms one-shot
    }

    // FIFO write: just log byte count (Serial Monitor will show it)
    if (state->current_reg == REG_FIFO) {
      printf("[LoRa] FIFO byte written: 0x%02X\n", data);
    }

    state->current_reg++;
    return 0x00;
  } else {
    uint8_t val = state->regs[state->current_reg];
    state->current_reg++;
    return val;
  }
}

static void spi_done(void *user_data) {
  chip_state_t *state = (chip_state_t *)user_data;
  state->first_byte = true; // ready for next transaction
  // Clear DIO0 after host has read IRQ flags
  if (pin_read(state->pin_dio0) == HIGH) {
    pin_write(state->pin_dio0, LOW);
    state->regs[REG_IRQ_FLAGS] = 0x00;
  }
}

void chip_init(void) {
  memset(&chip, 0, sizeof(chip));

  // Set key register defaults
  chip.regs[REG_VERSION]   = 0x12; // SX1278 version — critical for library init
  chip.regs[REG_OP_MODE]   = 0x80; // Sleep + LoRa mode
  chip.regs[REG_PA_CONFIG] = 0x4F;
  chip.regs[REG_IRQ_FLAGS] = 0x00;
  chip.first_byte          = true;
  chip.tx_pending          = false;

  chip.pin_dio0 = pin_init("DIO0", OUTPUT);
  chip.pin_rst  = pin_init("RST",  INPUT);
  chip.pin_nss  = pin_init("NSS",  INPUT);
  pin_write(chip.pin_dio0, LOW);

  spi_config_t spi_cfg = {
    .sck       = pin_init("SCK",  INPUT),
    .mosi      = pin_init("MOSI", INPUT),
    .miso      = pin_init("MISO", OUTPUT),
    .mode      = 0,
    .rx_data   = spi_rx,
    .done      = spi_done,
    .user_data = &chip,
  };
  chip.spi = spi_init(&spi_cfg);

  timer_config_t timer_cfg = {
    .callback  = pulse_dio0,
    .user_data = &chip,
  };
  chip.tx_done_timer = timer_init(&timer_cfg);

  printf("[LoRa SX1278] Custom chip initialized. RegVersion=0x12\n");
}