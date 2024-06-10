#include <WiFi.h>
#include <PubSubClient.h>
#include "base64.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "esp_camera.h"
#include <NeoSWSerial.h>

// Pilih model kamera
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

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

// Inisialisasi GPS Serial
NeoSWSerial gpsSerial(16, 17);

// Variabel untuk data lokasi
char date[11];
char time[11];
double latitude = 0.0, longitude = 0.0;

WiFiClient espClient;
PubSubClient client(espClient);

// Topik MQTT
const char* stream_topic = "stream/";
const char* open_stream_topic = "open_stream/"; 
const char* close_topic = "close_stream/";
const char* status_topic = "status/";
const char* alert_topic = "alert/";
const char* location_topic = "location/";

void publishStatus(const char* message);
void sendFrame();
void connectToMqtt();
void callback(char* topic, byte* payload, unsigned int length);
void setup_wifi();
void reconnect();
void updateGPS();
void parseGPRMC(char* sentence);
void sendLocation();
double convertToDecimalDegrees(char* coordinate, char direction);

void setup() {
  Serial.begin(115200);

  // Inisialisasi Buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  
  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("DMS ");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("ACTIVE");
  delay(8000);

  // Set baud rate for serial communication (default for Neo-M8N)
  gpsSerial.begin(9600);

  // Konfigurasi kamera
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
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
    publishStatus(errorMsg);
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

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  connectToMqtt();
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

  // Memperbarui data GPS
  updateGPS();
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
      client.subscribe(close_topic);
      client.subscribe(alert_topic);
      client.publish(open_stream_topic + id, "ESP32-CAM active");
    } else {
      char errorMsg[50];
      snprintf(errorMsg, 50, "Failed, rc=%d try again in 5 s", client.state());
      publishStatus(errorMsg);
      delay(5000);
    }
  }
}

void sendFrame() {
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    publishStatus("Image capture failed");
    return;
  }

  String imageFile = "data:image/jpeg;base64,";
  imageFile += base64::encode(fb->buf, fb->len);

  client.publish(String(stream_topic) + id, imageFile.c_str());

  esp_camera_fb_return(fb);

  client.publish(String(status_topic) + id, "Frame sent");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String incoming = "";
  for (int i = 0; i < length; i++) {
    incoming += (char)payload[i];
  }

  if (String(topic) == String(close_topic) + id) {
    if (incoming == "close") {
    publishStatus("Receive a message to close the stream");
    client.publish(String(status_topic) + id, "Stream closed");
    ESP.restart();
    }
  }

  if (String(topic) ==  String(alert_topic) + id) {
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

void updateGPS() {
  if (gpsSerial.available()) {
    char data = gpsSerial.read();

    if (data == '$') {
      char sentence[100];
      int index = 0;

      while (gpsSerial.available()) {
        data = gpsSerial.read();
        sentence[index++] = data;
        if (data == '\n' || index >= sizeof(sentence) - 1) {
          break;
        }
      }
      sentence[index] = '\0';

      if (strstr(sentence, "$GPRMC") != NULL) {
        parseGPRMC(sentence);
        sendLocation(); // Mengirimkan lokasi saat data GPS diterima
      }
    }
  }
}

void parseGPRMC(char* sentence) {
  char* fields[14];
  char* field = strtok(sentence, ",");
  int i = 0;
  while (field != NULL && i < 14) {
    fields[i++] = field;
    field = strtok(NULL, ",");
  }

  if (fields[2][0] != 'A') {
    Serial.println("Invalid data");
    return;
  }

  strncpy(date, fields[9], sizeof(date) - 1);
  date[sizeof(date) - 1] = '\0';

  strncpy(time, fields[1], sizeof(time) - 1);
  time[sizeof(time) - 1] = '\0';

  latitude = convertToDecimalDegrees(fields[3], fields[4][0]);
  longitude = convertToDecimalDegrees(fields[5], fields[6][0]);
}

void sendLocation() {
  char locationMessage[100];
  snprintf(locationMessage, 100, "{\"latitude\":%.6f,\"longitude\":%.6f}", latitude, longitude);
  client.publish(String(location_topic) + id, locationMessage);
}

double convertToDecimalDegrees(char* coordinate, char direction) {
  if (coordinate[0] == '\0') {
    return 0.0;
  }

  double degrees = atof(coordinate) / 100.0;
  double minutes = degrees - int(degrees);
  degrees = int(degrees) + minutes * 100.0 / 60.0;

  if (direction == 'S' || direction == 'W') {
    degrees *= -1;
  }

  return degrees;
}
