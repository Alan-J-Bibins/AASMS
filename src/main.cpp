#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

// Pins
const int servoPin = 3;
const int confirmButtonPin = 2;
const int addPotentiometerPin = A0;
const int subPotentiometerPin = A1;
const int bmpCS = 10;

// Objects
Adafruit_BMP280 bmp(bmpCS);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo ventServo;

// State
int oldButtonValue = HIGH;
float targetAltitude = -1;
bool targetSet = false;

// --- SIMULATION MODE ---
// Set to true to use Serial input instead of BMP280
// Set to false to use real BMP280 readings
const bool SIMULATION_MODE = true;
float simulatedAltitude = 0.0;

float getAltitude() {
    if (SIMULATION_MODE) {
        return simulatedAltitude;
    } else {
        return bmp.readAltitude();
    }
}

void setup()
{
    Serial.begin(9600);
    Serial.println(F("Initializing..."));

    pinMode(confirmButtonPin, INPUT_PULLUP);
    ventServo.attach(servoPin);
    ventServo.write(0);  // Start closed

    lcd.init();
    lcd.backlight();
    lcd.print("Welcome");

    if (!bmp.begin()) {
        Serial.println(F("Cannot find BMP280 Sensor"));
        lcd.setCursor(0, 1);
        lcd.print("Sensor error");
        while (1);
    }

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16,
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_500);

    if (SIMULATION_MODE) {
        Serial.println(F("*** SIMULATION MODE ON ***"));
        Serial.println(F("Type a number in Serial Monitor to set altitude."));
    }

    Serial.println(F("System Ready."));
}

void loop()
{
    // --- Simulation: read altitude from Serial Monitor ---
    if (SIMULATION_MODE && Serial.available()) {
    String input = Serial.readStringUntil('\n');  // reads until newline, discards it
    input.trim();                                  // removes any leftover \r or spaces
    simulatedAltitude = input.toFloat();
    Serial.print(F("Simulated altitude set to: "));
    Serial.println(simulatedAltitude);
}

    float altitude = getAltitude();

    int newButtonValue = digitalRead(confirmButtonPin);
    int addPotentiometerValue = analogRead(addPotentiometerPin);
    int subPotentiometerValue = analogRead(subPotentiometerPin);

    int positiveOffset = map(addPotentiometerValue, 0, 1023, 0, 1000);
    int negativeOffset = map(subPotentiometerValue, 0, 1023, 0, 1000);
    float previewAltitude = altitude + positiveOffset - negativeOffset;

    // --- Button: lock in target altitude ---
    if (newButtonValue != oldButtonValue) {
    if (newButtonValue == LOW) {
        targetAltitude = previewAltitude;
        targetSet = true;

        Serial.println(F("--- Target Confirmed ---"));
        Serial.print(F("Current Altitude : ")); Serial.println(altitude);
        Serial.print(F("Target Altitude  : ")); Serial.println(targetAltitude);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Target Set:");
        lcd.setCursor(0, 1);
        lcd.print(targetAltitude, 1);
        lcd.print(" m");
    } else {
        Serial.println(F("Button released."));
    }
    oldButtonValue = newButtonValue;  // ← must be HERE, inside the outer if
    delay(200);
}

    // --- Feedback loop ---
    if (targetSet) {
        if (altitude > targetAltitude) {
            ventServo.write(90);
            Serial.print(F("STATE: VENTING  | Alt: "));
        } else {
            ventServo.write(0);
            Serial.print(F("STATE: CLOSED   | Alt: "));
        }

        Serial.print(altitude, 1);
        Serial.print(F("m | Target: "));
        Serial.print(targetAltitude, 1);
        Serial.println(F("m"));

        // Live LCD update
        lcd.setCursor(0, 0);
        lcd.print("Alt:");
        lcd.print(altitude, 1);
        lcd.print("m   ");
        lcd.setCursor(0, 1);
        lcd.print("Tgt:");
        lcd.print(targetAltitude, 1);
        lcd.print("m   ");
    }

    delay(500);
}
