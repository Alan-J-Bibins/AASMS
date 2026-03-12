#include <stdio.h>
#include <stdlib.h>
#include "wokwi-api.h"

// Endereços dos registradores do BMP280
#define ID            0xD0  // Registrador de ID (retorna 0x58)
#define RESET         0xE0  // Registrador de reset
#define STATUS        0xF3  // Registrador de status
#define CTRL_MEAS     0xF4  // Controle de medição
#define CONFIG        0xF5  // Configuração
#define PRES_MSB      0xF7  // Pressão MSB
#define PRES_LSB      0xF8  // Pressão LSB
#define PRES_XLSB     0xF9  // Pressão XLSB
#define TEMP_MSB      0xFA  // Temperatura MSB
#define TEMP_LSB      0xFB  // Temperatura LSB
#define TEMP_XLSB     0xFC  // Temperatura XLSB

// Registradores de calibração
#define DIG_T1_L      0x88
#define DIG_T1_H      0x89
#define DIG_T2_L      0x8A
#define DIG_T2_H      0x8B
#define DIG_T3_L      0x8C
#define DIG_T3_H      0x8D
#define DIG_P1_L      0x8E
#define DIG_P1_H      0x8F
#define DIG_P2_L      0x90
#define DIG_P2_H      0x91
#define DIG_P3_L      0x92
#define DIG_P3_H      0x93
#define DIG_P4_L      0x94
#define DIG_P4_H      0x95
#define DIG_P5_L      0x96
#define DIG_P5_H      0x97
#define DIG_P6_L      0x98
#define DIG_P6_H      0x99
#define DIG_P7_L      0x9A
#define DIG_P7_H      0x9B
#define DIG_P8_L      0x9C
#define DIG_P8_H      0x9D
#define DIG_P9_L      0x9E
#define DIG_P9_H      0x9F

// Valores de calibração padrão
#define T1_DEFAULT    27504
#define T2_DEFAULT    26435
#define T3_DEFAULT    -1000
#define P1_DEFAULT    36477
#define P2_DEFAULT    -10685
#define P3_DEFAULT    3024
#define P4_DEFAULT    2855
#define P5_DEFAULT    140
#define P6_DEFAULT    -7
#define P7_DEFAULT    15500
#define P8_DEFAULT    -14600
#define P9_DEFAULT    6000

// Estrutura de estado do chip
typedef struct {
  pin_t cs;         // Chip select
  pin_t vcc;        // Alimentação
  spi_dev_t spi0;   // Dispositivo SPI
  uint8_t spi_buffer[1];  // Buffer SPI
  uint8_t nbyte;          // Contador de bytes
  uint8_t addr;           // Endereço atual
  uint8_t read_mode;      // 1 = leitura, 0 = escrita
  uint8_t ctrl_meas;      // Controle de medição
  uint8_t config;         // Configuração
  uint16_t dig_T1;        // Calibração T1
  int16_t  dig_T2;        // Calibração T2
  int16_t  dig_T3;        // Calibração T3
  uint16_t dig_P1;        // Calibração P1
  int16_t  dig_P2;        // Calibração P2
  int16_t  dig_P3;        // Calibração P3
  int16_t  dig_P4;        // Calibração P4
  int16_t  dig_P5;        // Calibração P5
  int16_t  dig_P6;        // Calibração P6
  int16_t  dig_P7;        // Calibração P7
  int16_t  dig_P8;        // Calibração P8
  int16_t  dig_P9;        // Calibração P9
  uint32_t temp_attr_id;     // Atributo TEMP
  uint32_t pres_attr_id;     // Atributo PRES
  uint32_t altitude_attr_id; // Atributo ALTITUDE
  int32_t t_fine;            // Valor fino de temperatura
} chip_state_t;

// Calcula pressão a partir da altitude
uint32_t calculate_pressure_from_altitude(int32_t altitude) {
  const uint32_t P0 = 101325; // Pressão ao nível do mar (Pa)
  const int32_t k = 12;       // 12 Pa por metro
  int32_t pressure = P0 - (altitude * k);
  if (pressure < 30000) pressure = 30000;   // Limite mínimo
  if (pressure > 110000) pressure = 110000; // Limite máximo
  return (uint32_t)pressure;
}

// Converte temperatura para valor bruto
uint32_t convert_temp_to_raw(chip_state_t *chip) {
  int32_t temp_input = (int32_t)attr_read(chip->temp_attr_id); // Lê TEMP em centigrados (ex.: 2500 = 25°C)
  int32_t var1 = (((temp_input * 5) / 128) * chip->dig_T2) / 100;
  int32_t var2 = (((temp_input * temp_input) / 16384) * chip->dig_T3) / 100;
  chip->t_fine = var1 + var2;
  int32_t raw_temp = (chip->t_fine * 5 + 128) / 256;
  return (uint32_t)raw_temp;
}

// Converte pressão para valor bruto
uint32_t convert_pressure_to_raw(chip_state_t *chip) {
  uint32_t pressure_input;
  int32_t pres_value = (int32_t)attr_read(chip->pres_attr_id);
  if (pres_value >= 30000 && pres_value <= 110000) {
    pressure_input = (uint32_t)pres_value; // Usa PRES se válido
  } else {
    int32_t altitude = (int32_t)attr_read(chip->altitude_attr_id);
    pressure_input = calculate_pressure_from_altitude(altitude); // Calcula a partir de ALTITUDE
  }
  
  convert_temp_to_raw(chip); // Calcula t_fine
  
  int64_t var1, var2, p;
  var1 = (int64_t)chip->t_fine - 128000;
  var2 = var1 * var1 * (int64_t)chip->dig_P6;
  var2 = var2 + ((var1 * (int64_t)chip->dig_P5) << 17);
  var2 = var2 + (((int64_t)chip->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)chip->dig_P3) >> 8) + ((var1 * (int64_t)chip->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)chip->dig_P1) >> 33;
  
  if (var1 == 0) return 0; // Evita divisão por zero
  
  p = 1048576 - pressure_input;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)chip->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)chip->dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)chip->dig_P7) << 4);
  
  return (uint32_t)p;
}

// Handler de mudança de pinos
void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;
  if (pin_read(chip->vcc) != HIGH) return;
  
  if (pin == chip->cs) {
    if (value == LOW) {
      chip->nbyte = 0;
      spi_start(chip->spi0, chip->spi_buffer, sizeof(chip->spi_buffer));
    } else {
      spi_stop(chip->spi0);
    }
  }
}

// Handler de transação SPI
void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t*)user_data;
  
  if (pin_read(chip->cs) == LOW && pin_read(chip->vcc) == HIGH) {
    if (chip->nbyte == 0) {
      chip->addr = chip->spi_buffer[0];
      chip->read_mode = (chip->spi_buffer[0] & 0x80) ? 1 : 0;
      if (chip->read_mode) {
        switch (chip->addr) {
          case ID: chip->spi_buffer[0] = 0x58; break;
          default: chip->spi_buffer[0] = 0x00; break;
        }
      }
    } else {
      if (chip->read_mode) {
        switch (chip->addr) {
          case DIG_T1_L: chip->spi_buffer[0] = chip->dig_T1 & 0xFF; break;
          case DIG_T1_H: chip->spi_buffer[0] = (chip->dig_T1 >> 8) & 0xFF; break;
          case DIG_T2_L: chip->spi_buffer[0] = chip->dig_T2 & 0xFF; break;
          case DIG_T2_H: chip->spi_buffer[0] = (chip->dig_T2 >> 8) & 0xFF; break;
          case DIG_T3_L: chip->spi_buffer[0] = chip->dig_T3 & 0xFF; break;
          case DIG_T3_H: chip->spi_buffer[0] = (chip->dig_T3 >> 8) & 0xFF; break;
          case DIG_P1_L: chip->spi_buffer[0] = chip->dig_P1 & 0xFF; break;
          case DIG_P1_H: chip->spi_buffer[0] = (chip->dig_P1 >> 8) & 0xFF; break;
          case DIG_P2_L: chip->spi_buffer[0] = chip->dig_P2 & 0xFF; break;
          case DIG_P2_H: chip->spi_buffer[0] = (chip->dig_P2 >> 8) & 0xFF; break;
          case DIG_P3_L: chip->spi_buffer[0] = chip->dig_P3 & 0xFF; break;
          case DIG_P3_H: chip->spi_buffer[0] = (chip->dig_P3 >> 8) & 0xFF; break;
          case DIG_P4_L: chip->spi_buffer[0] = chip->dig_P4 & 0xFF; break;
          case DIG_P4_H: chip->spi_buffer[0] = (chip->dig_P4 >> 8) & 0xFF; break;
          case DIG_P5_L: chip->spi_buffer[0] = chip->dig_P5 & 0xFF; break;
          case DIG_P5_H: chip->spi_buffer[0] = (chip->dig_P5 >> 8) & 0xFF; break;
          case DIG_P6_L: chip->spi_buffer[0] = chip->dig_P6 & 0xFF; break;
          case DIG_P6_H: chip->spi_buffer[0] = (chip->dig_P6 >> 8) & 0xFF; break;
          case DIG_P7_L: chip->spi_buffer[0] = chip->dig_P7 & 0xFF; break;
          case DIG_P7_H: chip->spi_buffer[0] = (chip->dig_P7 >> 8) & 0xFF; break;
          case DIG_P8_L: chip->spi_buffer[0] = chip->dig_P8 & 0xFF; break;
          case DIG_P8_H: chip->spi_buffer[0] = (chip->dig_P8 >> 8) & 0xFF; break;
          case DIG_P9_L: chip->spi_buffer[0] = chip->dig_P9 & 0xFF; break;
          case DIG_P9_H: chip->spi_buffer[0] = (chip->dig_P9 >> 8) & 0xFF; break;
          case CTRL_MEAS: chip->spi_buffer[0] = chip->ctrl_meas; break;
          case CONFIG: chip->spi_buffer[0] = chip->config; break;
          case STATUS: chip->spi_buffer[0] = 0x00; break;
          case TEMP_XLSB: {
            uint32_t temp_raw = convert_temp_to_raw(chip);
            chip->spi_buffer[0] = (temp_raw & 0xF) << 4;
            break;
          }
          case TEMP_LSB: {
            uint32_t temp_raw = convert_temp_to_raw(chip);
            chip->spi_buffer[0] = (temp_raw >> 4) & 0xFF;
            break;
          }
          case TEMP_MSB: {
            uint32_t temp_raw = convert_temp_to_raw(chip);
            chip->spi_buffer[0] = (temp_raw >> 12) & 0xFF;
            break;
          }
          case PRES_XLSB: {
            uint32_t pres_raw = convert_pressure_to_raw(chip);
            chip->spi_buffer[0] = (pres_raw & 0xF) << 4;
            break;
          }
          case PRES_LSB: {
            uint32_t pres_raw = convert_pressure_to_raw(chip);
            chip->spi_buffer[0] = (pres_raw >> 4) & 0xFF;
            break;
          }
          case PRES_MSB: {
            uint32_t pres_raw = convert_pressure_to_raw(chip);
            chip->spi_buffer[0] = (pres_raw >> 12) & 0xFF;
            break;
          }
          case ID: chip->spi_buffer[0] = 0x58; break;
          default: chip->spi_buffer[0] = 0x00; break;
        }
      } else {
        switch (chip->addr & 0x7F) {
          case (CTRL_MEAS & 0x7F): chip->ctrl_meas = chip->spi_buffer[0]; break;
          case (CONFIG & 0x7F): chip->config = chip->spi_buffer[0]; break;
          case (RESET & 0x7F):
            if (chip->spi_buffer[0] == 0xB6) {
              chip->ctrl_meas = 0x00;
              chip->config = 0x00;
            }
            break;
          default: break;
        }
      }
    }
    
    if (chip->read_mode && chip->nbyte > 0) chip->addr++;
    spi_start(chip->spi0, chip->spi_buffer, sizeof(chip->spi_buffer));
    chip->nbyte++;
  }
}

// Inicialização do chip
void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  
  chip->vcc = pin_init("VCC", INPUT);
  pin_init("GND", INPUT);
  chip->cs = pin_init("CS", INPUT);
  
  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = chip_pin_change,
    .user_data = chip,
  };
  pin_watch(chip->cs, &watch_config);
  
  const spi_config_t spi_config = {
    .sck = pin_init("SCK", INPUT),
    .mosi = pin_init("SDI", INPUT),
    .miso = pin_init("SDO", OUTPUT),
    .mode = 0,
    .done = chip_spi_done,
    .user_data = chip,
  };
  chip->spi0 = spi_init(&spi_config);
  
  chip->ctrl_meas = 0x00;
  chip->config = 0x00;
  
  chip->dig_T1 = T1_DEFAULT;
  chip->dig_T2 = T2_DEFAULT;
  chip->dig_T3 = T3_DEFAULT;
  chip->dig_P1 = P1_DEFAULT;
  chip->dig_P2 = P2_DEFAULT;
  chip->dig_P3 = P3_DEFAULT;
  chip->dig_P4 = P4_DEFAULT;
  chip->dig_P5 = P5_DEFAULT;
  chip->dig_P6 = P6_DEFAULT;
  chip->dig_P7 = P7_DEFAULT;
  chip->dig_P8 = P8_DEFAULT;
  chip->dig_P9 = P9_DEFAULT;
  
  chip->temp_attr_id = attr_init("TEMP", 2500);      // 25.00°C
  chip->pres_attr_id = attr_init("PRES", 101325);    // 101325 Pa
  chip->altitude_attr_id = attr_init("ALTITUDE", 0); // 0 metros
  
  chip->nbyte = 0;
  chip->addr = 0;
  chip->read_mode = 0;
  chip->t_fine = 0;
  
  printf("BMP280 inicializado.\n");
}

