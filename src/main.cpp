#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <WString.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp32-hal-gpio.h>

// --- Configuration ---
const char* wifi_ssid = "Honor10Lite";
const char* wifi_password = "AJBfifa2k20";
WebServer server(80);

const int servoPin = 17;
const int confirmButtonPin = 21;
const int addPotPin = 34, subPotPin = 35;
const int bmpCS = 5;
const int rs = 14, en = 13, d4 = 27, d5 = 26, d6 = 25, d7 = 33;

Adafruit_BMP280 bmp(bmpCS);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Servo ventServo;

// --- Global Flight State ---
volatile float currentAltitude = 0;
volatile float groundAltitude = 0;
volatile float targetAltitude = -1;
volatile bool targetSet = false;
volatile bool isLanding = false;
volatile int currentServoAngle = 0;
volatile float currentTemp = 0;
volatile float currentPressure = 0;

// PID & Physics Constants
float Kp = 1.2, Ki = 0.01, Kd = 0.5;
const float DESCENT_RATE = 0.4f; // Meters per second
const float GROUND_THRESHOLD = 2.0f;

TaskHandle_t PIDTaskHandle;

void sendCORSJson(int code, String content)
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(code, "application/json", content);
}

void handleRoot()
{
    float altAGL = currentAltitude - groundAltitude;
    if (altAGL < 0)
        altAGL = 0;

    String statusStr;
    if (isLanding) {
        statusStr = (altAGL < GROUND_THRESHOLD) ? "LANDED" : "LANDING";
    } else if (targetSet) {
        statusStr = "LOCKED";
    } else {
        statusStr = "IDLE";
    }

    // Full JSON Restoration
    String json = "{";
    json += "\"temp_c\":" + String(currentTemp, 1) + ",";
    json += "\"pressure_hpa\":" + String(currentPressure, 1) + ",";
    json += "\"alt_m\":" + String(currentAltitude, 2) + ",";
    json += "\"alt_agl\":" + String(altAGL, 2) + ",";
    json += "\"target_m\":" + String(targetAltitude, 2) + ",";
    json += "\"burner_deg\":" + String(currentServoAngle) + ",";
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
            isLanding = false; // Cancel landing if manual target is sent
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

// --- PID Task (Pinned to Core 1) ---
void PIDLoop(void* pvParameters)
{
    float lastError = 0, integral = 0;
    unsigned long lastTime = millis();

    // CRITICAL: This must be outside the loop to track time correctly
    static unsigned long lastDescendTick = 0;

    for (;;) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;

        // Sensor Refresh
        currentAltitude = bmp.readAltitude(1013.25);
        currentTemp = bmp.readTemperature();
        currentPressure = bmp.readPressure() / 100.0F;

        // --- LANDING RAMP LOGIC ---
        if (isLanding) {
            if (lastDescendTick == 0)
                lastDescendTick = now;

            if (now - lastDescendTick >= 1000) {
                if (targetAltitude > (groundAltitude + 0.1f)) {
                    targetAltitude -= DESCENT_RATE;
                    Serial.printf("DESCENDING: New Target: %.2f\n", targetAltitude);
                } else {
                    targetAltitude = groundAltitude;
                    Serial.println("LANDING COMPLETE: Target reached ground.");
                }
                lastDescendTick = now;
                Serial.printf("TICK: Target is now %.2f\n", targetAltitude);
            }
        } else {
            lastDescendTick = 0;
        }

        // --- PID CALCULATION ---
        if (targetSet && dt > 0) {
            float error = targetAltitude - currentAltitude;
            float pOut = Kp * error;

            if (abs(error) < 10.0f)
                integral += error * dt;
            float iOut = Ki * integral;
            float dOut = Kd * ((error - lastError) / dt);

            currentServoAngle = constrain((int)(pOut + iOut + dOut), 0, 90);
            ventServo.write(currentServoAngle);
            lastError = error;
        }

        lastTime = now;
        vTaskDelay(50 / portTICK_PERIOD_MS); // Run at 20Hz
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(confirmButtonPin, INPUT_PULLUP);

    ESP32PWM::allocateTimer(0);
    ventServo.setPeriodHertz(50);
    ventServo.attach(servoPin, 500, 2400);
    ventServo.write(0);

    lcd.begin(16, 2);
    if (!bmp.begin()) {
        Serial.println("BMP280 Search Fail");
        while (1)
            ;
    }

    delay(2000);
    groundAltitude = bmp.readAltitude(1013.25);
    targetAltitude = groundAltitude;

    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

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

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    server.begin();
    xTaskCreatePinnedToCore(PIDLoop, "PIDTask", 4096, NULL, 1, &PIDTaskHandle, 1);
}

void loop()
{
    server.handleClient();

    int addVal = analogRead(addPotPin);
    int subVal = analogRead(subPotPin);
    float previewAlt = currentAltitude + map(addVal, 0, 4095, 0, 10) - map(subVal, 0, 4095, 0, 10);

    static int lastBtn = HIGH;
    int btn = digitalRead(confirmButtonPin);
    if (btn == LOW && lastBtn == HIGH) {
        targetAltitude = previewAlt;
        targetSet = true;
        isLanding = false; // Manual button press kills auto-landing
        lcd.clear();
        lcd.print("MANUAL LOCK");
    }
    lastBtn = btn;

    // Display Update
    static unsigned long lastLCD = 0;
    if (millis() - lastLCD > 300) {
        lcd.setCursor(0, 0);
        float hgt = currentAltitude - groundAltitude;
        lcd.print("AGL: ");
        lcd.print(hgt < 0 ? 0.0f : hgt, 1);
        lcd.print("m  ");

        lcd.setCursor(0, 1);
        if (isLanding)
            lcd.print("MODE: LANDING  ");
        else {
            lcd.print(targetSet ? "Tgt: " : "Set: ");
            lcd.print(targetSet ? targetAltitude : previewAlt, 1);
            lcd.print("m  ");
        }
        lastLCD = millis();
    }
}
