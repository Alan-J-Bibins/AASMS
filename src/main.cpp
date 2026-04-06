#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <WString.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp32-hal-adc.h>
#include <esp32-hal-gpio.h>

// --- Configuration ---
const char* wifi_ssid = "Honor10Lite";
const char* wifi_password = "AJBfifa2k20";
WebServer server(80);

// NEW PINS APPLIED
const int servoPin = 27; // Data on D27
const int confirmButtonPin = 14; // Button on D14
const int landButtonPin = 13; // Button on D13
const int addPotPin = 32; // Pins 21/22 now used for I2C
const int subPotPin = 35;
const int trigPin = 18;
const int echoPin = 19;
const int obstacleLedPin = 26;
const int landingLedPin = 33;

// I2C Instances (Pins 21/22)
Adafruit_BMP280 bmp;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo ventServo;

// --- Global Flight State ---
volatile float currentAltitude = 0;
volatile float currentAltitudeAboveGround = 0;
volatile float groundAltitude = 0;
volatile float targetAltitude = -1;
volatile bool targetSet = false;
volatile bool isLanding = false;
volatile int currentServoAngle = 0;
volatile float currentTemp = 0;
volatile float currentPressure = 0;

// PID & Physics Constants (Kept exactly as original)
float Kp = 1.2, Ki = 0.01, Kd = 0.5;
const float DESCENT_RATE = 0.4f;
const float GROUND_THRESHOLD = 2.0f;

TaskHandle_t PIDTaskHandle;

void sendCORSJson(int code, String content)
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(code, "application/json", content);
}

void handleRoot()
{
    String statusStr;
    if (isLanding) {
        statusStr = (currentAltitudeAboveGround != -1 && currentAltitudeAboveGround < GROUND_THRESHOLD) ? "LANDED" : "LANDING";
    } else if (targetSet) {
        statusStr = "LOCKED";
    } else {
        statusStr = "IDLE";
    }

    String json = "{";
    json += "\"temp_c\":" + String(currentTemp, 1) + ",";
    json += "\"pressure_hpa\":" + String(currentPressure, 1) + ",";
    json += "\"alt_baro\":" + String(currentAltitude, 2) + ",";
    json += "\"alt_radar\":" + String(currentAltitudeAboveGround, 2) + ",";
    json += "\"target_asl\":" + String(targetAltitude, 2) + ",";
    json += "\"servo_deg\":" + String(currentServoAngle) + ",";
    json += "\"is_landing\":" + String(isLanding ? "true" : "false") + ",";
    json += "\"status\":\"" + statusStr + "\"";
    json += "}";

    sendCORSJson(200, json);
}

void handleSetTarget()
{
    if (server.hasArg("plain")) {
        JsonDocument doc;
        deserializeJson(doc, server.arg("plain"));
        float newTarget = doc["altitude"];

        if (newTarget >= 0) {
            targetAltitude = newTarget;
            targetSet = true;
            isLanding = false;
            sendCORSJson(200, "{\"success\":true,\"message\":\"Target altitude locked\"}");
        } else {
            sendCORSJson(400, "{\"success\":false,\"message\":\"Invalid altitude\"}");
        }
    }
}

void handleLandCommand()
{
    isLanding = true;
    targetSet = true;
    sendCORSJson(200, "{\"success\":true,\"message\":\"Landing sequence started\"}");
    Serial.println("WEB_CMD: Landing Initiated");
}

float readUltrasonic()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 25ms timeout (~4.2 meters max range)
    long duration = pulseIn(echoPin, HIGH, 25000);
    if (duration == 0)
        return -1.0f; // Out of range
    return (duration * 0.000343f) / 2.0f; // Returns meters
}

void PIDLoop(void* pvParameters)
{
    float lastError = 0, integral = 0;
    unsigned long lastTime = millis();
    static unsigned long lastDescendTick = 0;

    for (;;) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;

        // 1. Refresh Sensors
        currentAltitudeAboveGround = readUltrasonic();
        currentAltitude = bmp.readAltitude(1013.25);
        currentTemp = bmp.readTemperature();
        currentPressure = bmp.readPressure() / 100.0F;

        // 2. Landing Logic: Lower Baro Target until Ultrasonic takes over
        if (isLanding) {
            if (lastDescendTick == 0)
                lastDescendTick = now;
            if (now - lastDescendTick >= 1000) {
                // If radar is out of range (> 4m), keep dropping the Baro target
                if (currentAltitudeAboveGround < 0) {
                    targetAltitude -= DESCENT_RATE;
                }
                lastDescendTick = now;
            }
        } else {
            lastDescendTick = 0;
        }

        // 3. PID Calculation
        if (targetSet && dt > 0) {
            float error;

            // --- MODE A: Precision Landing (Radar Active) ---
            // If landing and we have a valid ground reading under 3.5 meters
            if (isLanding && currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 3.5f) {
                // Target is "0" (the ground). Error = Target - Current
                error = 0.0f - currentAltitudeAboveGround;
            }
            // --- MODE B: Barometric Cruise / High Altitude Descent ---
            else {
                error = targetAltitude - currentAltitude;

                // OBSTACLE AVOIDANCE: If cruising and something gets closer than 1.5m
                if (!isLanding && currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 1.5f) {
                    // We "add" to the error to force the PID to climb
                    error += (1.5f - currentAltitudeAboveGround) * 2.0f;
                }
            }

            // Standard PID Math (Matches your Kp, Ki, Kd)
            float pOut = Kp * error;
            if (abs(error) < 10.0f)
                integral += error * dt;
            float iOut = Ki * integral;
            float dOut = Kd * ((error - lastError) / dt);

            // Final Servo Output
            currentServoAngle = constrain((int)(pOut + iOut + dOut), 0, 90);

            // Touchdown Safety: If we are within 15cm of the ground, stop the motor
            if (isLanding && currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 0.15f) {
                currentServoAngle = 0;
            }

            ventServo.write(currentServoAngle);
            lastError = error;
        }

        lastTime = now;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    
    // 1. Diagnostics & Safety Pins
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW); // Status LED starts OFF
    pinMode(confirmButtonPin, INPUT_PULLUP);
    pinMode(landButtonPin, INPUT_PULLUP);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(obstacleLedPin, OUTPUT);
    pinMode(landingLedPin, OUTPUT);

    // 2. CRITICAL: Power Stabilization Delay
    // Gives the external 5V supply time to stop "ringing" or "bouncing"
    delay(2000); 

    // 3. I2C Bus & LCD (Low Power)
    // We start this first so the screen can tell us what's happening
    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("BOOT: I2C OK");
    delay(500);

    // 4. BMP280 Sensor
    if (!bmp.begin(0x76)) {
        Serial.println("BMP280 Fail");
        lcd.setCursor(0,1);
        lcd.print("BMP280 ERROR!");
        while (1); // Halt if sensor missing
    }
    groundAltitude = bmp.readAltitude(1013.25);
    lcd.setCursor(0,1);
    lcd.print("BARO: CALIBRATED");
    delay(1000);

    // 5. WiFi Connection (High Power Spike)
    // We do this BEFORE the servo to ensure the radio gets priority current
    lcd.clear();
    lcd.print("WIFI: CONNECTING");
    lcd.setCursor(0,1);
    lcd.print(wifi_ssid);

    WiFi.begin(wifi_ssid, wifi_password);
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
        delay(500);
        Serial.print(".");
        retryCount++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(2, HIGH); // Blue LED ON = WiFi Connected
        lcd.clear();
        lcd.print("WIFI: CONNECTED");
        lcd.setCursor(0,1);
        lcd.print(WiFi.localIP());
    } else {
        lcd.clear();
        lcd.print("WIFI: TIMEOUT");
        lcd.setCursor(0,1);
        lcd.print("CHECK HOTSPOT");
    }
    delay(1500); // Wait for WiFi surge to settle

    // 6. Network Services
    if (MDNS.begin("balloon")) {
        MDNS.addService("http", "tcp", 80);
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_POST, handleSetTarget);
    server.on("/land", HTTP_POST, handleLandCommand);

    auto cors = []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        server.send(204);
    };
    server.on("/", HTTP_OPTIONS, cors);
    server.on("/set", HTTP_OPTIONS, cors);
    server.on("/land", HTTP_OPTIONS, cors);
    server.begin();

    // 7. Servo Attachment (Mechanical Surge)
    // We do this last because the "twitch" on attach draws significant Amps
    ESP32PWM::allocateTimer(0);
    ventServo.setPeriodHertz(50);
    ventServo.attach(servoPin, 500, 2400);
    ventServo.write(0);

    lcd.clear();
    lcd.print("SYSTEM READY");
    delay(1000);

    // 8. Start Flight Control Task
    xTaskCreatePinnedToCore(PIDLoop, "PIDTask", 4096, NULL, 1, &PIDTaskHandle, 1);
}

void loop()
{
    server.handleClient();

    // 1. Altitude Preview Logic
    int addVal = analogRead(addPotPin);
    int subVal = analogRead(subPotPin);
    // Standardizing the map to 0-50m for finer control during demo
    float previewAlt = currentAltitude + map(addVal, 0, 4095, 0, 50) - map(subVal, 0, 4095, 0, 50);

    // 2. Button 1: Lock Target
    static int lastConfirm = HIGH;
    int confirmBtn = digitalRead(confirmButtonPin);
    if (confirmBtn == LOW && lastConfirm == HIGH) {
        targetAltitude = previewAlt;
        targetSet = true;
        isLanding = false;
        lcd.clear();
        lcd.print("MANUAL LOCK");
        delay(200); // Small debounce
    }
    lastConfirm = confirmBtn;

    // 3. Button 2: Land Command
    static int lastLand = HIGH;
    int landBtn = digitalRead(landButtonPin);
    if (landBtn == LOW && lastLand == HIGH) {
        isLanding = true;
        targetSet = true;
        lcd.clear();
        lcd.print("INIT LANDING");
        delay(200);
    }
    lastLand = landBtn;

    // 4. LED Indicators
    // Obstacle LED (D26)
    digitalWrite(obstacleLedPin, (currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 1.5f) ? HIGH : LOW);

    // Landing LED (D33)
    digitalWrite(landingLedPin, (isLanding && currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 0.2f) ? HIGH : LOW);

    // 5. Wind Gust Detection
    static float lastPressure = 0;
    static unsigned long lastWindCheck = 0;
    bool windWarningActive = false;

    if (millis() - lastWindCheck > 100) {
        float pressureDiff = abs(currentPressure - lastPressure);
        if (pressureDiff > 0.5f && lastPressure != 0) {
            windWarningActive = true;
        }
        lastPressure = currentPressure;
        lastWindCheck = millis();
    }

    // 6. LCD Update (Every 300ms)
    static unsigned long lastLCD = 0;
    if (millis() - lastLCD > 300) {
        lcd.setCursor(0, 0);
        float hgt = currentAltitude - groundAltitude;
        lcd.print("AGL: ");
        lcd.print(currentAltitudeAboveGround < 0 ? "INF" : String(currentAltitudeAboveGround, 1));
        lcd.print("m    "); // Spaces to clear old digits

        lcd.setCursor(0, 1);
        if (windWarningActive) {
            lcd.print("!! WIND GUST !! ");
        } else if (isLanding) {
            lcd.print(currentAltitudeAboveGround < 0.2f ? "MODE: LANDED    " : "MODE: LANDING   ");
        } else {
            lcd.print(targetSet ? "Tgt: " : "Set: ");
            lcd.print(targetSet ? targetAltitude : previewAlt, 1);
            lcd.print("m    ");
        }
        lastLCD = millis();
    }
}
