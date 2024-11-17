#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>

#include "Secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Fonts/Org_01.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>

// Define the pin where the MQ135, Vibration, DHT11 sensors, and ACS712 sensor are connected
#define AO_PIN 34 // MQ135 gas sensor
#define relay 14
const int vibrationPin = 4; // Vibration sensor pin
#define DHTPIN 16           // DHT11 sensor pin
#define DHTTYPE DHT11  
int load = 32;    
int led = 27; 
int buzz = 13; 
int wifi = 25; // LED for gas detection alert
int threshold = 3; // Threshold for triggering the LED
int cnt = 0; // Counter for gas detection
#define ACS712_PIN 35  // ACS712 sensor pin (Analog Input)

// Calibration for ACS712 sensor (for a 5A sensor)
float zeroCurrentVoltage = 2.5; // Voltage at zero current (2.5V typically)
float sensitivity = 0.185;      // Sensitivity for ACS712 5A: 185mV/A

// OLED Display
#define i2c_Address 0x3c // initialize with the I2C addr 0x3C (Typically eBay OLED's)
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

DHT dht(DHTPIN, DHTTYPE);

// AWS IoT topics
#define AWS_IOT_PUBLISH_TOPIC "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

// Global variables
float humidity;
float temperature;
int analogValue;
float smoke;
bool vibrationDetected;
float current;

WiFiClientSecure net;
PubSubClient client(net);

void connectAWS() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.println("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(wifi, HIGH);
        delay(500);
        digitalWrite(wifi, LOW);
        delay(500);
        Serial.print(".");
    }

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    
    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);

    Serial.println("Connecting to AWS IoT");
    while (!client.connect(THINGNAME)) {
        Serial.print(".");
        delay(100);
    }

    if (!client.connected()) {
        Serial.println("AWS IoT Timeout!");
        return;
    }

    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.println("AWS IoT Connected!");
}

void publishMessage() {
    StaticJsonDocument<200> doc;
    doc["humidity"] = humidity;
    doc["temperature"] = temperature;
    doc["gas_level"] = smoke;
    doc["vibration"] = vibrationDetected ? "YES" : "NO";
    doc["current"] = current; // Publish current reading
    
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
    Serial.print("Incoming message: ");
    Serial.println(topic);

    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    const char* message = doc["message"];
    Serial.println(message);
}

void setup() {
    Serial.begin(115200);

    // Initialize sensors
    dht.begin();
    pinMode(relay, OUTPUT);
    pinMode(buzz, OUTPUT);
    pinMode(load, OUTPUT);
    pinMode(AO_PIN, INPUT);
    pinMode(led, OUTPUT);
    pinMode(wifi, OUTPUT);
    pinMode(vibrationPin, INPUT);
    pinMode(ACS712_PIN, INPUT); // Initialize ACS712 sensor pin

    // Initialize OLED display
    if (!display.begin(i2c_Address, true)) {
        Serial.println("SH1106 allocation failed");
        for (;;);
    }

    connectAWS();
}

void loop() {
    // Read sensors
    analogValue = analogRead(AO_PIN);
    smoke = map(analogValue, 0, 4095, 0, 100); // Convert analog value to a percentage
    vibrationDetected = digitalRead(vibrationPin);
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    // Read ACS712 sensor for current measurement
    int sensorValue = analogRead(ACS712_PIN);
    float voltage = sensorValue * (3.3 / 4095.0); // Convert ADC reading to voltage (3.3V reference, 12-bit ADC)
    current = (voltage - zeroCurrentVoltage) / sensitivity; // Calculate current (in amps)

    digitalWrite(wifi, HIGH);
    digitalWrite(load, HIGH);

    // Gas detection logic with LED alert
    if (smoke >= 50) {  // If gas is detected (MQ sensor typically gives HIGH when gas is present)
        cnt++;  // Increment counter for every gas detection
        if (cnt >= threshold) {  // If gas detected continuously for 'threshold' iterations
            digitalWrite(led, HIGH);
            digitalWrite(buzz, HIGH); 
            digitalWrite(load, LOW); 
            digitalWrite(relay, HIGH); // Turn on relay and buzzer
        }
    } else {
        cnt = 0;  // Reset counter if no gas detected
        digitalWrite(led, LOW);
        digitalWrite(buzz, LOW); 
        digitalWrite(load, HIGH);
        digitalWrite(relay, LOW); // Turn off relay
    }

    // Handle error reading from DHT sensor
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    // Print to Serial Monitor
    Serial.print("Analog value from MQ135: ");
    Serial.println(smoke);
    Serial.print("Vibration: ");
    Serial.println(vibrationDetected ? "Detected" : "No vibration");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
    Serial.print("Current: ");
    Serial.print(current);
    Serial.println(" A");
    Serial.println(cnt);
    Serial.println(sensorValue);

    // Display on OLED
    display.clearDisplay();
    display.setTextSize(0);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Gas Level: ");
    display.println(smoke);
    display.print("Vibration: ");
    display.println(vibrationDetected ? "YES" : "NO");
    display.print("Humidity: ");
    display.printf("%.2f%%\n", humidity);
    display.print("Temperature: ");
    display.printf("%.1f C\n", temperature);
    display.print("Current: ");
    display.printf("%.3f A", current);
    display.display();

    // Publish to AWS IoT
    publishMessage();
    
    client.loop();
    delay(1000);
}
