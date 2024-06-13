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

// Define function prototypes
void setup_wifi();
void connectToMqtt();
void callback(char* topic, byte* payload, unsigned int length);
void sendFrame();
void sendLocation();

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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 8;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    char errorMsg[50];
    snprintf(errorMsg, 50, "Camera init failed with error 0x%x", err);
    Serial.println(errorMsg);
    delay(1000);
    ESP.restart();
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

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
  if (millis() - lastFrameTime >= 1000 / 15) {
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
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to capture image from camera.");
    return;
  } else {
    Serial.println("Image captured successfully.");
  }

  // Encode frame buffer in Base64
  String base64Frame = base64::encode(fb->buf, fb->len);
  Serial.println("Frame encoded in Base64.");
  Serial.println(base64Frame);

  // Check payload size
  if (base64Frame.length() > 25000000) { 
    Serial.println("Payload size exceeds limit. Frame not sent.");
    esp_camera_fb_return(fb);
    return;
  }

  // Construct the payload with the Base64 encoded frame
  String payload = + base64Frame;

  // Publish payload to the MQTT topic
  if (client.publish("stream/", payload.c_str())) {
    Serial.println("Frame sent successfully.");
  } else {
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
      client.publish("status/", "Connected to MQTT");
      client.subscribe("close_stream/");
      client.subscribe("alert/");
      char topic[50];
      snprintf(topic, sizeof(topic), "open_stream/%s", id);
      client.publish(topic, id);    } else {
      char errorMsg[50];
      snprintf(errorMsg, 50, "Failed, rc=%d try again in 5 s", client.state());
      client.publish("status/", errorMsg);
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
