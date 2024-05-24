#include <WiFi.h> 
#include <PubSubClient.h>  // komunikasi MQTT
#include <Camera.h>
#include <Ublox.h>  // gps
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <base64.h>

// Define WiFi for connection between system and web
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";

// Define MQTT broker credentials
const char* mqtt_server = "MQTT_BROKER_ADDRESS";
const char* mqtt_username = "MQTT_USERNAME";
const char* mqtt_password = "c241-ms01";

// Define MQTT topics
const char* topic_photo = "photo";
const char* topic_video = "video";
const char* topic_location = "location";
const char* topic_alert = "alert";

// Define buzzer and LCD pins
const int buzzerPin = 13;
const int lcdAddress = 0x27;

// Define LCD display
// LiquidCrystal_I2C lcd(lcdAddress, 16, 2);
LiquidCrystal_I2C lcd(0x3F,16,2);

// Define Ublox GPS module
Ublox gps(Serial1);

// Define WiFi client and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Define data buffer for image and video frames
uint8_t buffer[1024];

// Define variables for driver behavior detection results
bool isDrowsy = false;
bool isDistracted = false;

// Function to handle incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  if (String(topic) == topic_alert) {
    if (message == "drowsy") {
      isDrowsy = true;
    } else if (message == "distracted") {
      isDistracted = true;
    } else {
      isDrowsy = false;
      isDistracted = false;
    }
  }
}

// Function to reconnect WiFi
void reconnectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    WiFi.begin(ssid, password);
  }
  Serial.println("WiFi reconnected");
}

// Function to reconnect MQTT
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(topic_alert);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize buzzer pin as output
  pinMode(buzzerPin, OUTPUT);

  // Initialize LCD display
  lcd.init();
  lcd.backlight();

  // Initialize WiFi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Initialize MQTT client
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Initialize Ublox GPS module
  gps.begin();
}

void loop() {
  // Check if WiFi connection is still alive
  if (!WiFi.isConnected()) {
    reconnectWiFi();
  }

  // Check if MQTT connection is lost
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();  // Process incoming messages and maintain connection

  // Capture image frame from ESP32 Cam
  if (camera.grabFrame()) {
    // Convert image frame to JPEG format
    camera.jpgCompress(buffer, 1024);

    // Convert JPEG to Base64
    String imageBase64 = base64::encode(buffer, camera.getSize());

    // Publish image frame to MQTT topic
    client.publish(topic_photo, imageBase64.c_str());
    Serial.println("Image published in Base64 format");
  }

  // Capture video frame from ESP32 Cam
  if (camera.grabFrame()) {
    // Convert video frame to raw RGB data
    camera.getRawRGB(buffer, camera.getSize());

    // Convert raw RGB to Base64
    String videoBase64 = base64::encode(buffer, camera.getSize());

    // Publish video frame to MQTT topic
    client.publish(topic_video, videoBase64.c_str());
    Serial.println("Video frame published in Base64 format");
  }

  // Read GPS data from Ublox module
  if (gps.isConnected()) {
    // Get latitude and longitude
    float latitude = gps.getLatitude();
    float longitude = gps.getLongitude();

    // Create JSON string for location data
    String locationData = "{\"latitude\": " + String(latitude) + ", \"longitude\": " + String(longitude) + "}";

    // Publish location data to MQTT topic
    client.publish(topic_location, locationData.c_str());
    Serial.println("Location published");
  }

  // Check for driver behavior detection results from server
  if (isDrowsy || isDistracted) {
    // Activate buzzer
    digitalWrite(buzzerPin, HIGH);

    // Display alert message on LCD
    lcd.clear();
    lcd.print("Driver terdeteksi ");
    if (isDrowsy) {
      lcd.print("ngantuk");
    } else if (isDistracted) {
      lcd.print("lengah");
    }
  } else {
    // Deactivate buzzer
    digitalWrite(buzzerPin, LOW);

    // Clear LCD display
    lcd.clear();
  }
}
