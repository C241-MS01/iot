#include <WiFi.h>
#include <PubSubClient.h>

// Alamat dan port server MQTT
const char* mqtt_server = "34.101.43.219";
const int mqtt_port = 1883;

// Username dan password MQTT
const char* mqtt_user = "admin";
const char* mqtt_password = "c241-ms01";

// Konfigurasi WiFi
const char* ssid = "Wokwi-GUEST";
const char* password = "";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

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

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  if (String(topic) == "alert" && message == "drowsy") {
    Serial.println("Drowsiness detected!");
    
  }

  
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("Connected");
      client.publish("location", "ESP32 connected");
      client.subscribe("alert");    // Subscribe to the "alert" topic
      client.subscribe("location"); // Subscribe to the "location" topic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) { // Publish message every 2 seconds
    lastMsg = now;
    snprintf(msg, MSG_BUFFER_SIZE, "test masuk gak");
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("location", msg); // Publish message to the "location" topic
  }
}
