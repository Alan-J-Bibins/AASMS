#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp32-hal-gpio.h>
#include <ESPmDNS.h>

const char* ap_ssid = "surjo";
const char* ap_password = "1234567890";
WebServer server(80);

const int servoPin = 17;
const int confirmButtonPin = 21;
const int addPotPin = 34, subPotPin = 35;
const int bmpCS = 5;
const int rs = 14, en = 13, d4 = 27, d5 = 26, d6 = 25, d7 = 33;

Adafruit_BMP280 bmp(bmpCS);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Servo ventServo;

volatile float currentAltitude = 0;
volatile float targetAltitude = -1;
volatile bool targetSet = false;
volatile int currentServoAngle = 0;
volatile float currentTemp = 0;
volatile float currentPressure = 0;

// PID Constants
float Kp = 1.2, Ki = 0.01, Kd = 0.5;

TaskHandle_t PIDTaskHandle;

void handleRoot()
{
    String json = "{\n";
    json += "  \"temp_c\": " + String(currentTemp) + ",\n";
    json += "  \"pressure_hpa\": " + String(currentPressure) + ",\n";
    json += "  \"alt_m\": " + String(currentAltitude) + ",\n";
    json += "  \"target_m\": " + String(targetAltitude) + ",\n";
    json += "  \"burner_deg\": " + String(currentServoAngle) + ",\n";
    json += "  \"status\": \"" + String(targetSet ? "LOCKED" : "IDLE") + "\"\n";
    json += "}\n";
    server.send(200, "application/json", json);
}

void PIDLoop(void* pvParameters)
{
    float lastError = 0;
    float integral = 0;
    unsigned long lastTime = millis();

    for (;;) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;

        currentAltitude = bmp.readAltitude(1013.25);
        currentTemp = bmp.readTemperature();
        currentPressure = bmp.readPressure() / 100.0F;

        if (targetSet && dt > 0) {
            float error = targetAltitude - currentAltitude;

            float pOut = Kp * error;
            if (abs(error) < 10.0)
                integral += error * dt;
            float iOut = Ki * integral;
            float dOut = Kd * ((error - lastError) / dt);

            float totalOutput = pOut + iOut + dOut;
            currentServoAngle = constrain((int)totalOutput, 0, 90);
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

    pinMode(confirmButtonPin, INPUT_PULLUP);
    ESP32PWM::allocateTimer(0);
    ventServo.setPeriodHertz(50);
    ventServo.attach(servoPin, 500, 2400);
    ventServo.write(0);

    lcd.begin(16, 2);
    if (!bmp.begin()) {
        Serial.println("BMP Fail");
        while (1)
            ;
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_500);

    Serial.printf("Connecting to %s ", ap_ssid);
    WiFi.begin(ap_ssid, ap_password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi Connected!");

    if (!MDNS.begin("balloon")) {
        Serial.println("Error setting up MDNS responder!");
    } else {
        Serial.println("mDNS responder started: http://balloon.local");
    }

    MDNS.addService("http", "tcp", 80);

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.on("/", handleRoot);
    server.begin();

    xTaskCreatePinnedToCore(PIDLoop, "PIDTask", 4096, NULL, 1, &PIDTaskHandle, 1);
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
}

void loop()
{
    server.handleClient();

    int addVal = analogRead(addPotPin);
    int subVal = analogRead(subPotPin);
    float previewAlt = currentAltitude + map(addVal, 0, 4095, 0, 100) - map(subVal, 0, 4095, 0, 100);

    static int lastBtn = HIGH;
    int btn = digitalRead(confirmButtonPin);
    if (btn == LOW && lastBtn == HIGH) {
        targetAltitude = (previewAlt < 0) ? 0 : previewAlt;
        targetSet = true;
        lcd.clear();
        lcd.print("LOCKED!");
    }
    lastBtn = btn;

    static unsigned long lastLCD = 0;
    if (millis() - lastLCD > 250) {
        lcd.setCursor(0, 0);
        lcd.print("Alt: ");
        lcd.print(currentAltitude, 1);
        lcd.print("m  ");
        lcd.setCursor(0, 1);
        lcd.print(targetSet ? "Tgt: " : "Set: ");
        lcd.print(targetSet ? targetAltitude : previewAlt, 1);
        lcd.print("m  ");
        lastLCD = millis();
    }

    delay(10);
}
