#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// WiFi and MQTT credentials
const char* ssid = "Galaxy_A33_5G_5F04";
const char* password = "jijit4332";
const char* mqtt_server = "34.101.71.74";
const int mqtt_port = 1883;
const char* mqtt_user = "admin";
const char* mqtt_password = "c241-ms01";
const char* id = "45c1a8d1-b0e9-4c91-a177-603e3a63ebab";
const int buzzerPin = 13;

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// LCD configuration
#define SDA 21   
#define SCL 22
#define COLUMNS 16
#define ROWS 2
LiquidCrystal_I2C lcd(0x27, COLUMNS, ROWS);

// GPS configuration
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);  // Use UART2 for GPS

// Function prototypes
void setup_wifi();
void connectToMqtt();
void callback(char* topic, byte* payload, unsigned int length);
void sendLocation();

void setup() {
  Serial.begin(115200);

  // Initialize buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Initialize LCD
  Wire.begin(SDA, SCL);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DMS ");
  delay(3000);
  lcd.setCursor(0, 1);
  lcd.print("ACTIVE");
  delay(3000);

  // Initialize GPS
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX2 = GPIO 16, TX2 = GPIO 17

  // Connect to WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  connectToMqtt();
}

void loop() {
  if (!client.connected()) {
    connectToMqtt();
  }
  client.loop();

  // Read GPS data and send location if available
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      Serial.println("GPS data received. Sending location...");
      sendLocation();
    }
  }
}

void sendLocation() {
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Create a JSON document
    StaticJsonDocument<200> doc;
    doc["latitude"] = latitude;
    doc["longitude"] = longitude;

    // Serialize the JSON document to a string
    char payload[256];
    serializeJson(doc, payload);

    // Construct the topic
    char topicloc[50];
    snprintf(topicloc, sizeof(topicloc), "location/%s", id);

    // Publish the JSON payload
    if (client.publish(topicloc, payload)) {
      Serial.println("Location sent successfully.");
    } else {
      Serial.println("Failed to send location.");
    }
  } else {
    Serial.println("GPS location invalid");
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectToMqtt() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32CAM", mqtt_user, mqtt_password)) {
      char alert[50];
      snprintf(alert, sizeof(alert), "alert/%s", id);
      client.subscribe(alert);
      client.subscribe("close_stream");
      client.publish("open_stream", id);    
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String incoming = "";
  for (unsigned int i = 0; i < length; i++) {
    incoming += (char)payload[i];
  }

  // Restart the ESP32 if a close_stream message is received with the correct ID
  if (String(topic) == "close_stream") {
    if (incoming == id) {
      ESP.restart();
    }
  }

  char alert[50];
  snprintf(alert, sizeof(alert), "alert/%s", id);

  // Trigger alert if an alert message is received
  if (String(topic) == alert) {
    if (incoming == "bottle" || incoming == "cigarette" || incoming == "phone" || incoming == "smoke" || incoming == "vape" || incoming == "not focus") {
      Serial.println("Alert detected: " + incoming);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Alert!");
      lcd.setCursor(0, 1);
      lcd.print(incoming);
      digitalWrite(buzzerPin, HIGH);
      delay(5000);
      digitalWrite(buzzerPin, LOW);
    }
  }
}
