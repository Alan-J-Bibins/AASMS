#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <ESP32Servo.h>

// --- PIN MAPPING (Verified for your current diagram) ---
const int servoPin = 17;
const int confirmButtonPin = 21;
const int addPotPin = 34;
const int subPotPin = 35;
const int bmpCS = 5;
const int rs = 14, en = 13, d4 = 27, d5 = 26, d6 = 25, d7 = 33;

// --- PID CONSTANTS (Tuning required for physical balloon) ---
float Kp = 1.2;  
float Ki = 0.01; 
float Kd = 0.5;  

// PID Variables
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// Hardware Objects
Adafruit_BMP280 bmp(bmpCS);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Servo ventServo;

// State Variables
float targetAltitude = -1;
bool targetSet = false;
int lastButtonState = HIGH;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("--- BALLOON PID CONTROLLER STARTING ---"));

    // 1. Pins & Peripheral Setup
    pinMode(confirmButtonPin, INPUT_PULLUP);
    
    // ESP32 Servo Specifics
    ESP32PWM::allocateTimer(0);
    ventServo.setPeriodHertz(50);
    ventServo.attach(servoPin, 500, 2400); 
    ventServo.write(0); // Start closed

    // 2. LCD Init
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("System Ready");

    // 3. BMP280 Init
    if (!bmp.begin()) {
        Serial.println(F("CRITICAL ERROR: BMP280 NOT FOUND"));
        lcd.setCursor(0,1);
        lcd.print("Sensor Error!");
        while(1); 
    }

    // High precision settings for altitude tracking
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

    lastTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Time in seconds

    // --- 1. SENSOR DATA ---
    float currentAlt = bmp.readAltitude(1013.25);
    
    // Read Potentiometers (0-4095 range)
    int addVal = analogRead(addPotPin);
    int subVal = analogRead(subPotPin);
    
    // Preview logic: Adjusting range to 100m for easier tuning
    float previewAlt = currentAlt + map(addVal, 0, 4095, 0, 100) - map(subVal, 0, 4095, 0, 100);
    if (previewAlt < 0) previewAlt = 0;

    // --- 2. BUTTON DEBOUNCE & LOCK ---
    int currentButtonState = digitalRead(confirmButtonPin);
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        targetAltitude = previewAlt;
        targetSet = true;
        integral = 0; // Reset integral on new target to prevent "jump"
        Serial.printf("TARGET LOCKED: %.1fm\n", targetAltitude);
        lcd.clear();
        lcd.print("LOCKED!");
        delay(200); // Simple debounce
    }
    lastButtonState = currentButtonState;

    // --- 3. PID CALCULATION ---
    if (targetSet && dt > 0) {
        float error = targetAltitude - currentAlt;

        // Proportional
        float pOut = Kp * error;

        // Integral (with anti-windup: only accumulate if close to target)
        if (abs(error) < 10.0) { 
            integral += error * dt;
        }
        float iOut = Ki * integral;

        // Derivative
        float dOut = Kd * ((error - lastError) / dt);

        float totalOutput = pOut + iOut + dOut;

        // Map output to Servo: 0 (Closed) to 90 (Full Burn)
        // If error is negative (too high), output will be < 0, constrain to 0.
        int servoAngle = constrain((int)totalOutput, 0, 90);
        ventServo.write(servoAngle);

        // Telemetry
        Serial.printf("ALT: %.1fm | TGT: %.1fm | ERR: %.1f | SERVO: %d\n", 
                      currentAlt, targetAltitude, error, servoAngle);
        
        lastError = error;
    }

    // --- 4. LCD UPDATE (Every 200ms to avoid flicker) ---
    static unsigned long lastLCDUpdate = 0;
    if (currentTime - lastLCDUpdate > 200) {
        lcd.setCursor(0, 0);
        lcd.print("Alt: "); lcd.print(currentAlt, 1); lcd.print("m   ");
        lcd.setCursor(0, 1);
        if (targetSet) {
            lcd.print("Tgt: "); lcd.print(targetAltitude, 1); lcd.print("m   ");
        } else {
            lcd.print("Set: "); lcd.print(previewAlt, 1); lcd.print("m   ");
        }
        lastLCDUpdate = currentTime;
    }

    lastTime = currentTime;
    delay(50); // Loop frequency ~20Hz
}
