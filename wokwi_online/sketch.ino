#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <LiquidCrystal_I2C.h>

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
LiquidCrystal_I2C lcd(0x27, 16, 2);

float groundPressure;
void setup() 
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Balloon Avionics System Starting");

  if (bmp.begin(0x76)) {
    Serial.println("BMP280 OK");
  } else {
    Serial.println("BMP280 NOT FOUND");
  }

  if (mpu.begin()) {
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 NOT FOUND");
  }

  float groundPressure = 1013.25;

  lcd.init();
  lcd.backlight();
}

void loop() 
{
  //Serial.println(bmp.readPressure() / 100.0);

  //float temp = bmp.readTemperature();
  //float altitude = bmp.readAltitude(groundPressure);

  float pressure = bmp.readPressure() / 100.0F;
float temp     = bmp.readTemperature();
float altitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));
Serial.println("------Telemetry------");

Serial.print("Pressure: "); Serial.println(pressure);
Serial.print("Altitude: "); Serial.println(altitude);
Serial.print("Temp: ");     Serial.println(temp);

  // sensors_event_t a,g,t;
  // mpu.getEvent(&a,&g,&t);

  // Serial.println("------Telemetry------");

  // Serial.print("Altitude: ");
  // Serial.print(altitude);
  // Serial.println(" m");

  // Serial.print("Temperature: ");
  // Serial.print(temp);
  // Serial.println(" C");

  // Serial.print("Accel X: ");
  // Serial.println(a.acceleration.x);

  // Serial.print("Accel Y: ");
  // Serial.println(a.acceleration.y);

  // Serial.print("Accel Z: ");
  // Serial.println(a.acceleration.z);

  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Alt:");
  // lcd.print(altitude);

  // lcd.setCursor(0,1);
  // lcd.print("Temp:");
  // lcd.print(temp);

  delay(1000);
}
