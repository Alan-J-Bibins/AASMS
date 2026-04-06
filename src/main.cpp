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

const char* wifi_ssid = "Honor10Lite";
const char* wifi_password = "AJBfifa2k20";
WebServer server(80);

const int servoPin = 27;
const int confirmButtonPin = 14;
const int landButtonPin = 13;
const int addPotPin = 32;
const int subPotPin = 35;
const int trigPin = 18;
const int echoPin = 19;
const int obstacleLedPin = 26;
const int landingLedPin = 33;

Adafruit_BMP280 bmp;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo ventServo;

volatile float currentAltitude = 0;
volatile float currentAltitudeAboveGround = 0;
volatile float groundAltitude = 0;
volatile float targetAltitude = -1;
volatile bool targetSet = false;
volatile bool isLanding = false;
volatile int currentServoAngle = 0;
volatile float currentTemp = 0;
volatile float currentPressure = 0;

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

    long duration = pulseIn(echoPin, HIGH, 25000);
    if (duration == 0)
        return -1.0f;
    return (duration * 0.000343f) / 2.0f;
}

void PIDLoop(void* pvParameters)
{
    float lastError = 0, integral = 0;
    unsigned long lastTime = millis();
    static unsigned long lastDescendTick = 0;

    for (;;) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;

        currentAltitudeAboveGround = readUltrasonic();
        currentAltitude = bmp.readAltitude(1013.25);
        currentTemp = bmp.readTemperature();
        currentPressure = bmp.readPressure() / 100.0F;

        if (isLanding) {
            if (lastDescendTick == 0)
                lastDescendTick = now;
            if (now - lastDescendTick >= 1000) {

                if (currentAltitudeAboveGround < 0) {
                    targetAltitude -= DESCENT_RATE;
                }
                lastDescendTick = now;
            }
        } else {
            lastDescendTick = 0;
        }

        if (targetSet && dt > 0) {
            float error;

            if (isLanding && currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 3.5f) {

                error = 0.0f - currentAltitudeAboveGround;
            }

            else {
                error = targetAltitude - currentAltitude;

                if (!isLanding && currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 1.5f) {

                    error += (1.5f - currentAltitudeAboveGround) * 2.0f;
                }
            }

            float pOut = Kp * error;
            if (abs(error) < 10.0f)
                integral += error * dt;
            float iOut = Ki * integral;
            float dOut = Kd * ((error - lastError) / dt);

            currentServoAngle = constrain((int)(pOut + iOut + dOut), 0, 90);

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

    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    pinMode(confirmButtonPin, INPUT_PULLUP);
    pinMode(landButtonPin, INPUT_PULLUP);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(obstacleLedPin, OUTPUT);
    pinMode(landingLedPin, OUTPUT);

    delay(2000);

    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("BOOT: I2C OK");
    delay(500);

    if (!bmp.begin(0x76)) {
        Serial.println("BMP280 Fail");
        lcd.setCursor(0, 1);
        lcd.print("BMP280 ERROR!");
        while (1)
            ;
    }
    groundAltitude = bmp.readAltitude(1013.25);
    lcd.setCursor(0, 1);
    lcd.print("BARO: CALIBRATED");
    delay(1000);

    lcd.clear();
    lcd.print("WIFI: STARTING AP");

    WiFi.setTxPower(WIFI_POWER_11dBm);

    if (WiFi.softAP("Balloon-Control", "password123")) {
        digitalWrite(2, HIGH);
        IPAddress myIP = WiFi.softAPIP();

        lcd.clear();
        lcd.print("AP: ACTIVE");
        lcd.setCursor(0, 1);
        lcd.print(myIP.toString());
        Serial.print("AP IP address: ");
        Serial.println(myIP);
    } else {
        lcd.clear();
        lcd.print("AP: FAILED");
    }

    delay(1500);

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

    ESP32PWM::allocateTimer(0);
    ventServo.setPeriodHertz(50);
    ventServo.attach(servoPin, 500, 2400);
    ventServo.write(0);

    lcd.clear();
    lcd.print("SYSTEM READY");
    delay(1000);

    xTaskCreatePinnedToCore(PIDLoop, "PIDTask", 4096, NULL, 1, &PIDTaskHandle, 1);
}

void loop()
{
    server.handleClient();

    int addVal = analogRead(addPotPin);
    int subVal = analogRead(subPotPin);

    float previewAlt = currentAltitude + map(addVal, 0, 4095, 0, 50) - map(subVal, 0, 4095, 0, 50);

    static int lastConfirm = HIGH;
    int confirmBtn = digitalRead(confirmButtonPin);
    if (confirmBtn == LOW && lastConfirm == HIGH) {
        targetAltitude = previewAlt;
        targetSet = true;
        isLanding = false;
        lcd.clear();
        lcd.print("MANUAL LOCK");
        delay(200);
    }
    lastConfirm = confirmBtn;

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

    digitalWrite(obstacleLedPin, (currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 1.5f) ? HIGH : LOW);

    digitalWrite(landingLedPin, (isLanding && currentAltitudeAboveGround > 0 && currentAltitudeAboveGround < 0.2f) ? HIGH : LOW);

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

    static unsigned long lastLCD = 0;
    if (millis() - lastLCD > 300) {
        lcd.setCursor(0, 0);
        float hgt = currentAltitude - groundAltitude;
        lcd.print("AGL: ");
        lcd.print(currentAltitudeAboveGround < 0 ? "INF" : String(currentAltitudeAboveGround, 1));
        lcd.print("m    ");

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
