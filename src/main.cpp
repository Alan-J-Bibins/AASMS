#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Wire.h>

// Pins assigned on the arduino
const int confirmButtonPin = 2;
const int addPotentiometerPin = A0;
const int subPotentiometerPin = A1;
const int bmpCS = 10;

// Initalizing objects
Adafruit_BMP280 bmp(bmpCS);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
    Serial.begin(9600);
    Serial.println(F("Initalizing..."));

    pinMode(confirmButtonPin, INPUT);

    lcd.init();
    lcd.backlight();
    lcd.print("Welcome");

    if (!bmp.begin()) {
        Serial.println(F("Cannot find BMP280 Sensor"));
        lcd.setCursor(0, 1);
        lcd.print("Sensor error");
        while (1)
            ;
    }

    // Optional: Configure Sensor Sampling
    // You can set how often the sensor filters noise for the balloon's ascent.
    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2, // Temp oversampling
        Adafruit_BMP280::SAMPLING_X16, // Pressure oversampling
        Adafruit_BMP280::FILTER_X16, // Filtering for smoothness
        Adafruit_BMP280::STANDBY_MS_500
    );

    Serial.println(F("System Ready."));
}

void loop()
{
    int addPotentiometerValue = analogRead(addPotentiometerPin);
    int subPotentiometerValue = analogRead(subPotentiometerPin);

    // Lets keep the map range max value as 1000 meters
    int positiveOffset = map(addPotentiometerValue, 0, 1023, 0, 1000);
    int negativeOffset = map(subPotentiometerValue, 0, 1023, 0, 1000);

    float altitude = bmp.readAltitude();
    float newAltitude = altitude + positiveOffset - negativeOffset;

    if (digitalRead(confirmButtonPin) == HIGH) {
        Serial.println(F("Button has been pressed"));
        Serial.print(F("Current: "));
        Serial.println(altitude);
        Serial.print(F("New: "));
        Serial.println(newAltitude);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Target Set: ");
        lcd.setCursor(0, 1);
        lcd.print(newAltitude);
        lcd.print(" m");

        // We don't want the button to trigger a million times do we
        delay(200);
    }

}
