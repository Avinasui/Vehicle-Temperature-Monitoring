#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include "time.h"
#include "Secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Fonts/Org_01.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>

// Define the pin where the MQ135, Vibration, DHT11 sensors, and ACS712 sensor are connected
#define AO_PIN 33 // MQ135 gas sensor
#define relay 14
const int vibrationPin = 4; 
const int lm35Pin = 33; // Vibration sensor pin
#define SENSOR_PIN 34 // Sound sensor pin

int load = 32;    
int led = 27; 
int buzz = 13; 
int wifi = 25; // LED for gas detection alert
int threshold = 3; // Threshold for triggering the LED
int cnt = 0; // Counter for gas detection
const int sensorIn = 35; // ACS712 sensor pin (Analog Input)

int mVperAmp = 185;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
// OLED Display
#define i2c_Address 0x3c // initialize with the I2C addr 0x3C (Typically eBay OLED's)
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// AWS IoT topics
#define AWS_IOT_PUBLISH_TOPIC "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

// Global variables
float humidity;
float temperature;
int analogValue;
bool vibrationDetected;
char timestamp[20];
float soundLevelDb; // Sound level in decibels

WiFiClientSecure net;
PubSubClient client(net);

// Vibration counting
int vibrationCount = 0;
unsigned long lastVibrationTime = 0; // Time for counting vibrations
unsigned long lastPublishTime = 0;   // Time for publishing data

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
    doc["timestamp"] = timestamp;
    doc["vibrations"] = vibrationCount; // Number of vibrations in 30 seconds
    doc["temperature"] = temperature;
    doc["current"] = AmpsRMS; // Publish current reading
    doc["power"] = Watt;
    doc["sound_level"] = soundLevelDb; // Publish sound level

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);

    // Reset vibration count after publishing
    vibrationCount = 0;
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
    configTime(19800, 0, "pool.ntp.org");

    // Initialize sensors
    pinMode(relay, OUTPUT);
    pinMode(buzz, OUTPUT);
    pinMode(load, OUTPUT);
    pinMode(AO_PIN, INPUT);
    pinMode(led, OUTPUT);
    pinMode(wifi, OUTPUT);
    pinMode(vibrationPin, INPUT);
    pinMode(sensorIn, INPUT);
    pinMode(lm35Pin, INPUT); // Initialize ACS712 sensor pin
    pinMode(SENSOR_PIN, INPUT); // Initialize sound sensor pin

    // Initialize OLED display
    if (!display.begin(i2c_Address, true)) {
        Serial.println("SH1106 allocation failed");
        for (;;);
    }

    connectAWS();
}

void loop() {
    // Read sensors
    analogValue = digitalRead(AO_PIN);
    vibrationDetected = digitalRead(vibrationPin);

    // Increment vibration count if detected
    if (vibrationDetected) {
        vibrationCount++;
    }
    
    temperature = readLM35Temperature();

    // Read ACS712 sensor for current measurement
    Voltage = getVPP();
    VRMS = (Voltage / 2.0) * 0.707;   // RMS voltage
    AmpsRMS = ((VRMS * 1000) / mVperAmp) - 1.5; // Adjusted for error
    digitalWrite(wifi, HIGH);
    digitalWrite(load, HIGH);
    if (analogValue== HIGH) {
        cnt++;
        if (cnt >= threshold) {
            digitalWrite(led, HIGH);
            digitalWrite(buzz, HIGH); 
            digitalWrite(load, LOW); 
            digitalWrite(relay, HIGH);
        }
    } else {
        cnt = 0;
        digitalWrite(led, LOW);
        digitalWrite(buzz, LOW); 
        digitalWrite(load, HIGH);
        digitalWrite(relay, LOW);
    }
    // Sound sensor decibel reading
    int soundAnalog = analogRead(SENSOR_PIN);
    soundLevelDb = map(soundAnalog, 0, 4095, 30, 120); // Map analog value to dB

    Watt = (AmpsRMS * 240 / 1.2); // Calculated Power

    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        strftime(timestamp, 20, "%Y-%m-%d %H:%M:%S", &timeinfo);
    }

    // Publish to AWS IoT every 30 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= 30000) {
        publishMessage();
        lastPublishTime = currentTime;
    }
    Serial.print("Vibration: ");
    Serial.println(vibrationCount);
    Serial.println("Temperature: ");
    Serial.print(temperature);
    Serial.println("C");
    Serial.print("Current: ");
    Serial.print(AmpsRMS);
    Serial.print(" A");
    Serial.print("Power: ");
    Serial.println(Watt);
    Serial.print("Sound Level: ");
    Serial.println(soundLevelDb);

    // Display on OLED
    display.clearDisplay();
    display.setTextSize(0);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Vibrations: ");
    display.println(vibrationCount);
    display.print("Temperature: ");
    display.printf("%.1f C\n", temperature);
    display.print("Current: ");
    display.printf("%.3f A\n", AmpsRMS);
    display.print("Power: ");
    display.printf("%d W\n", Watt);
    display.print("Sound: ");
    display.printf("%.1f dB\n", soundLevelDb);
    display.display();

    client.loop();
    delay(1000);
}
// Function to get peak-to-peak voltage
float getVPP() {
    float result;
    int readValue;                
    int maxValue = 0;             
    int minValue = 4096;          
    
    uint32_t start_time = millis();
    while((millis() - start_time) < 1000) { 
        readValue = analogRead(sensorIn);
        if (readValue > maxValue) maxValue = readValue;
        if (readValue < minValue) minValue = readValue;
    } 
    result = ((maxValue - minValue) * 3.3) / 4096.0;
    return result;
}
float readLM35Temperature() {
  int av = analogRead(lm35Pin);  // Read analog value from sensor
  float vol = av * (3.3 / 4096.0); // Convert to voltage
  float temperatureC = (vol * 10);  // Calculate temperature in Celsius
  return temperatureC;
}
