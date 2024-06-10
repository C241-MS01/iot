#include <WiFi.h>
#include <PubSubClient.h>
#include "base64.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "esp_camera.h"

// Konfigurasi WiFi
const char* ssid = "Galaxy_A33_5G_5F04";
const char* password = "jijit4332";

// Konfigurasi MQTT
const char* mqtt_server = "34.101.43.219";
const int mqtt_port = 1883;
const char* mqtt_user = "admin";
const char* mqtt_password = "c241-ms01";
const char* id = "45c1a8d1-b0e9-4c91-a177-603e3a63ebab";

// Pin Buzzer
const int buzzerPin = 13;

// Inisialisasi LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

WiFiClient espClient;
PubSubClient client(espClient);

// Topik MQTT
const char* stream_topic = "stream/";
const char* close_topic = "close_stream/";
const char* status_topic = "status/";
const char* alert_topic = "alert/";

void setup() {
  Serial.begin(115200);

  // Inisialisasi Buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);

  connectToMqtt();

  // Konfigurasi kamera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = GPIO_NUM_5; // IO5
  config.pin_d1 = GPIO_NUM_18; // IO18
  config.pin_d2 = GPIO_NUM_19; // IO19
  config.pin_d3 = GPIO_NUM_21; // IO21
  config.pin_d4 = GPIO_NUM_36; // IO36
  config.pin_d5 = GPIO_NUM_39; // IO39
  config.pin_d6 = GPIO_NUM_34; // IO34
  config.pin_d7 = GPIO_NUM_35; // IO35
  config.pin_xclk = GPIO_NUM_0; // IO0
  config.pin_pclk = GPIO_NUM_22; // IO22
  config.pin_vsync = GPIO_NUM_25; // IO25
  config.pin_href = GPIO_NUM_23; // IO23
  config.pin_sscb_sda = GPIO_NUM_26; // IO26
  config.pin_sscb_scl = GPIO_NUM_27; // IO27
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Inisialisasi kamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastFrameTime = 0;
  if (millis() - lastFrameTime >= 1000 / 15) { // 15 FPS
    lastFrameTime = millis();
    sendFrame();
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
    Serial.println("Menghubungkan ke MQTT...");
    if (client.connect("ESP32CAM", mqtt_user, mqtt_password)) {
      Serial.println("Terhubung ke MQTT");
      client.subscribe(close_topic);
      client.subscribe(alert_topic);

      // Ketika terhubung, kirimkan UUID ke topik 'open_stream'
      client.publish(String("open_stream/") + id, "ESP32-CAM aktif");
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

void sendFrame() {
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture gambar gagal");
    return;
  }

  String imageFile = "data:image/jpeg;base64,";
  imageFile += base64::encode(fb->buf, fb->len);

  // Kirim frame ke topik 'stream' + id
  client.publish(String(stream_topic) + id, imageFile.c_str());

  esp_camera_fb_return(fb);

  client.publish(String(status_topic) + id, "Frame terkirim");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String incoming = "";
  for (int i = 0; i < length; i++) {
    incoming += (char)payload[i];
  }

  if (String(topic) == String(close_topic) + id) {
    Serial.println("Menerima pesan untuk menutup stream");
    client.publish(String(status_topic) + id, "Stream ditutup");
    ESP.restart();
  }

  if (String(topic) == String(alert_topic) + id) {
    if (incoming == "drowsy" || incoming == "yawning" || incoming == "using_handphone") {
      Serial.println("Alert detected: " + incoming);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Alert!");
      lcd.setCursor(0, 1);
      lcd.print(incoming);
      digitalWrite(buzzerPin, HIGH);
      delay(5000); // Buzzer menyala selama 5 detik
      digitalWrite(buzzerPin, LOW);
    }
  }
}

void publishStatus(const char* message) {
  client.publish(String(status_topic) + id, message);
}
