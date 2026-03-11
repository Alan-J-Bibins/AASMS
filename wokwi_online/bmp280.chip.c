#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define BMP280_ADDR 0x76
#define CALIBRATION_DATA 0x88
#define TEMP_DATA 0xFA
#define PRESS_DATA 0xF7
#define CTRL_MEAS 0xF4
#define CONFIG 0xF5
#define RESET 0xE0

#define T1 27504
#define T2 26435
#define T3 50


enum state{
  temp,
  press
};

typedef struct {
  uint32_t temp;
  uint32_t press;
  uint8_t reg_address;
} chip_state_t;

bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (address == BMP280_ADDR) {
    return true;
  }
  return false;
}

uint8_t on_i2c_read(void *user_data) 
{
  chip_state_t *chip = (chip_state_t *)user_data;
  static uint8_t base_addr = 0xFA;
  static int counter = 0;
  int init = 0;

  if (chip->reg_address != 0xD0) {
    if (chip->reg_address == base_addr) {
      ((init += 1) != 0) ? counter++ : counter;
    } else {
      counter = 0;
      base_addr = chip->reg_address;
    }
  }

  uint8_t modified_reg_addr = chip->reg_address + counter;

  if (modified_reg_addr > 0xFC && base_addr == 0xFA) {
    modified_reg_addr -= 3;
  }

  if (modified_reg_addr > 0xF9 && base_addr == 0xF7) {
    modified_reg_addr -= 3;
  }

  uint8_t result = 0xFF;

 static int flight_time = 0;
static int phase = 0;       // 0=ascent,1=burst,2=descent
static float altitude = 0;  // meters

float ascentRate = 5;       // m/s
float descentRate = 5;      // m/s

// flight update (assuming loop dt ~1s)
flight_time++;

// update altitude based on phase
if (phase == 0) {              // ascent
    altitude += ascentRate;
    if (altitude >= 30000) phase = 1;  // burst at 30 km
}
else if (phase == 1) {         // burst
    phase = 2;
}
else if (phase == 2) {         // descent
    altitude -= descentRate;
    if (altitude < 0) altitude = 0;
}

// convert altitude → pressure (Pa)
float pressurePa = 101325 * pow(1 - altitude / 44330.0, 5.255);

// optional: convert temperature realistically
float tempC;
if (altitude < 11000)
    tempC = 15 - 0.0065 * altitude;       // troposphere
else
    tempC = -56.5;                        // stratosphere

// convert to raw BMP280 format (approximation)
static uint32_t rawPress;
static uint32_t rawTemp = 519888;;

rawPress = (uint32_t)(pressurePa * 16);        // BMP280 raw uses ~Pa*16
//rawTemp  = (uint32_t)((tempC + 273.15) * 100); // BMP280 raw temp units


  switch (modified_reg_addr) {

    case 0xD0:
      result = 0x58;
      break;

    /* calibration data */
    case CALIBRATION_DATA + 0: result = 0x70; break;
    case CALIBRATION_DATA + 1: result = 0x6B; break;
    case CALIBRATION_DATA + 2: result = 0x43; break;
    case CALIBRATION_DATA + 3: result = 0x67; break;
    case CALIBRATION_DATA + 4: result = 0x18; break;
    case CALIBRATION_DATA + 5: result = 0xFC; break;
    case CALIBRATION_DATA + 6: result = 0x7D; break;
    case CALIBRATION_DATA + 7: result = 0x8E; break;
    case CALIBRATION_DATA + 8: result = 0x43; break;
    case CALIBRATION_DATA + 9: result = 0xD6; break;
    case CALIBRATION_DATA + 10: result = 0xD0; break;
    case CALIBRATION_DATA + 11: result = 0x0B; break;
    case CALIBRATION_DATA + 12: result = 0x27; break;
    case CALIBRATION_DATA + 13: result = 0x0B; break;
    case CALIBRATION_DATA + 14: result = 0x8C; break;
    case CALIBRATION_DATA + 15: result = 0x00; break;
    case CALIBRATION_DATA + 16: result = 0xF9; break;
    case CALIBRATION_DATA + 17: result = 0xFF; break;
    case CALIBRATION_DATA + 18: result = 0x8C; break;
    case CALIBRATION_DATA + 19: result = 0x3C; break;
    case CALIBRATION_DATA + 20: result = 0xF8; break;
    case CALIBRATION_DATA + 21: result = 0xC6; break;
    case CALIBRATION_DATA + 22: result = 0x70; break;
    case CALIBRATION_DATA + 23: result = 0x17; break;

    /* pressure registers */
    case PRESS_DATA:
      result = (rawPress >> 12) & 0xFF;
      break;

    case PRESS_DATA + 1:
      result = (rawPress >> 4) & 0xFF;
      break;

    case PRESS_DATA + 2:
      result = (rawPress & 0x0F) << 4;
      break;

    /* temperature registers */
    case TEMP_DATA:
      result = (rawTemp >> 12) & 0xFF;
      break;

    case TEMP_DATA + 1:
      result = (rawTemp >> 4) & 0xFF;
      break;

    case TEMP_DATA + 2:
      result = (rawTemp & 0x0F) << 4;
      break;

    default:
      result = 0xFF;
      break;
  }

  return result;
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;
 // printf("ADDRESS: %x\n",data);
  chip->reg_address = data;
  return true;
}

void on_i2c_disconnect(void *user_data) {
}

/*void update_temperature(chip_state_t *chip) {
  // Fetch the temperature value from Wokwi control
  chip->temp = attr_read(temp_attr_id);
}

void update_pressure(chip_state_t *chip) {
  // Fetch the pressure value from Wokwi control
  chip->press = attr_read(press_attr_id);
}*/


void chip_init() {
  static chip_state_t chip;
  
  const i2c_config_t i2c_config = {
    .address = BMP280_ADDR,
    .scl = pin_init("SCL", INPUT_PULLUP),
    .sda = pin_init("SDA", INPUT_PULLUP),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
    .user_data = &chip,
  };

  i2c_init(&i2c_config);
  chip.reg_address = 0x00;
  printf("BMP280 initialized!\n");
  chip.temp = attr_init("temp",10000);

  printf("True");
  chip.press = attr_init("press",10000);

  

}

/*void chip_loop() {
  static chip_state_t chip; // Use the global or passed chip state

  printf("Reached chip_loop\n");

  // Update temperature and pressure values
  update_temperature(&chip);
  update_pressure(&chip);

  // Print the updated values
  printf("Updated Temperature: %u\n", chip.temp);
  printf("Updated Pressure: %u\n", chip.press);
}*/

