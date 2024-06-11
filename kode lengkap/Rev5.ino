#include <WiFi.h>
#include <PubSubClient.h>
#include "base64.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "esp_camera.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Pilih model kamera
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Define constants
const char* ssid = "Galaxy_A33_5G_5F04";
const char* password = "jijit4332";
const char* mqtt_server = "34.101.43.219";
const int mqtt_port = 1883;
const char* mqtt_user = "admin";
const char* mqtt_password = "c241-ms01";
const char* id = "45c1a8d1-b0e9-4c91-a177-603e3a63ebab";
const int buzzerPin = 13;
const int FPS = 15;

// Define function prototypes
void setup_wifi();
void connectToMqtt();
void callback(char* topic, byte* payload, unsigned int length);
void sendFrame();
void sendLocation();
void publishStatus(const char* message);
void rebootDevice();

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// GPS and LCD
TinyGPSPlus gps;
HardwareSerial GPSSerial(2); // Ensure GPS serial is set correctly
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);

  // Initialize buzzer and LCD
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DMS ");
  delay(3000);
  lcd.setCursor(0, 1);
  lcd.print("ACTIVE");
  delay(3000);

  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Failed to initialize camera. Rebooting...");
    rebootDevice();
  }

  // Initialize GPS
  GPSSerial.begin(9600);

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

  static unsigned long lastFrameTime = 0;
  if (millis() - lastFrameTime >= 1000 / FPS) {
    lastFrameTime = millis();
    Serial.println("Sending frame...");
    sendFrame();
  }

  // Read GPS data
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      Serial.println("GPS data received. Sending location...");
      sendLocation();
    }
  }
}

void sendFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    publishStatus("Image capture failed");
    Serial.println("Failed to capture image from camera.");
    return;
  } else {
    Serial.println("Image captured successfully.");
  }

  // Encode frame buffer in Base64
  String base64Frame = base64::encode(fb->buf, fb->len);
  Serial.println("Frame encoded in Base64.");
  Serial.println(base64Frame);

  // Construct the payload with the Base64 encoded frame
  String payload = "data:image/jpeg;base64," + base64Frame;

  // Publish payload to the MQTT topic
  if (client.publish("stream/", payload.c_str())) {
    publishStatus("Frame sent successfully.");
    Serial.println("Frame sent successfully.");
  } else {
    publishStatus("Failed to send frame.");
    Serial.println("Failed to send frame.");
  }

  esp_camera_fb_return(fb);
}

void sendLocation() {
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    String payload = String(latitude, 6) + "," + String(longitude, 6);

    if (client.publish("location/", payload.c_str())) {
      publishStatus("Location sent successfully.");
      Serial.println("Location sent successfully.");
    } else {
      publishStatus("Failed to send location.");
      Serial.println("Failed to send location.");
    }
  } else {
    publishStatus("GPS location invalid");
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
    publishStatus("Connecting to MQTT...");
    if (client.connect("ESP32CAM", mqtt_user, mqtt_password)) {
      publishStatus("Connected to MQTT");
      client.subscribe("close_stream/");
      client.subscribe("alert/");
      char topic[100];
      snprintf(topic, sizeof(topic), "open_stream/%s", id);
      client.publish(topic, id);
    } else {
      char errorMsg[50];
      snprintf(errorMsg, 50, "Failed, rc=%d try again in 5 s", client.state());
      publishStatus(errorMsg);
      delay(5000);
    }
  }
}



void callback(char* topic, byte* payload, unsigned int length) {
  String incoming = "";
  for (int i = 0; i < length; i++) {
    incoming += (char)payload[i];
  }

  if (String(topic) == "close_stream/") {
    publishStatus("Receive a message to close the stream");
    client.publish("status/", "Stream closed");
    ESP.restart();
  }

  if (String(topic) == "alert/") {
    if (incoming == "drowsy" || incoming == "yawning" || incoming == "using_handphone") {
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

void publishStatus(const char* message) {
  client.publish("status/", message);
}

void rebootDevice() {
  // Melakukan reboot perangkat secara lunak
  ESP.restart();
}
