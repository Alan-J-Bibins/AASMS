// Balloon System - Diagnostic Sketch v6
// Key insight: SD card shares SPI with LoRa (pins 11/12/13)
// SD CS (pin 10) must be held HIGH during LoRa transactions
// BMP280 on I2C (SDI=SDA=A4, SCK=SCL=A5), address 0x77 (SDO floating=HIGH)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>

#define PIN_SD_CS      10
#define PIN_LORA_NSS   A2
#define PIN_LORA_RST   A1
#define PIN_LORA_DIO0  2
#define PIN_GPS_RX     3
#define PIN_GPS_TX     4

SoftwareSerial  gpsSerial(PIN_GPS_RX, PIN_GPS_TX);
Adafruit_BMP280 bmp;   // I2C constructor

void scanI2C() {
  Serial.println("--- I2C Scan ---");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  None");
  Serial.println("----------------");
}

void testLoRa() {
  Serial.println("--- LoRa SPI Test ---");

  // Hold SD CS HIGH so it doesn't interfere
  pinMode(PIN_SD_CS,    OUTPUT);
  pinMode(PIN_LORA_NSS, OUTPUT);
  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_SD_CS,    HIGH);
  digitalWrite(PIN_LORA_NSS, HIGH);
  digitalWrite(PIN_LORA_RST, HIGH);
  delay(100);

  // Reset LoRa
  digitalWrite(PIN_LORA_RST, LOW);
  delay(10);
  digitalWrite(PIN_LORA_RST, HIGH);
  delay(100);

  SPI.begin();
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

  // Make sure SD is deselected
  digitalWrite(PIN_SD_CS, HIGH);

  // Read RegVersion (0x42)
  digitalWrite(PIN_LORA_NSS, LOW);
  delayMicroseconds(500);
  SPI.transfer(0x42);
  delayMicroseconds(500);
  byte ver = SPI.transfer(0x00);
  delayMicroseconds(500);
  digitalWrite(PIN_LORA_NSS, HIGH);

  SPI.endTransaction();
  SPI.end();

  Serial.print("  RegVersion: 0x"); Serial.print(ver, HEX);
  Serial.println(ver == 0x12 ? " -> OK" : " -> FAILED (expected 0x12)");
  Serial.println("---------------------");
}

void testBMP() {
  Serial.println("--- BMP280 I2C Test ---");
  // Try 0x77 first (SDO=HIGH), then 0x76 (SDO=LOW)
  uint8_t addr = 0;
  if      (bmp.begin(0x77, 0x12)) addr = 0x77;
  else if (bmp.begin(0x76, 0x12)) addr = 0x76;
  else if (bmp.begin(0x77))       addr = 0x77;
  else if (bmp.begin(0x76))       addr = 0x76;

  if (addr) {
    Serial.print("  BMP280 OK at 0x"); Serial.println(addr, HEX);
    Serial.print("  Temp:     "); Serial.print(bmp.readTemperature());       Serial.println(" C");
    Serial.print("  Pressure: "); Serial.print(bmp.readPressure() / 100.0F); Serial.println(" hPa");
    Serial.print("  Altitude: "); Serial.print(bmp.readAltitude(1013.25));   Serial.println(" m");
  } else {
    Serial.print("  FAILED. SensorID=0x"); Serial.println(bmp.sensorID(), HEX);
    Serial.println("  Hint: check SDI->A4, SCK->A5 in diagram.json");
  }
  Serial.println("-----------------------");
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("=== Diagnostic Boot v6 ===");

  Wire.begin();
  scanI2C();
  testLoRa();
  testBMP();

  gpsSerial.begin(9600);
  Serial.println("--- GPS (5s) ---");
  uint32_t start = millis();
  while (millis() - start < 5000) {
    if (gpsSerial.available()) Serial.write(gpsSerial.read());
  }
  Serial.println("\n=== Diagnostic Complete ===");
}

void loop() {
  while (gpsSerial.available()) Serial.write(gpsSerial.read());
}
