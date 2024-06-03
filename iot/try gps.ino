#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Inisialisasi objek TinyGPS++
TinyGPSPlus gps;

// Inisialisasi serial hardware untuk GPS
HardwareSerial SerialGPS(1);

// Definisi pin untuk RX dan TX
#define RXD2 16
#define TXD2 17

void setup() {
  // Inisialisasi serial untuk debugging
  Serial.begin(115200);

  // Inisialisasi serial untuk GPS
  SerialGPS.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Menunggu inisialisasi
  Serial.println("Mencari sinyal GPS...");
}

void loop() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
    
    if (gps.location.isUpdated()) {
      Serial.print("Latitude : ");
      Serial.println(gps.location.lat(), 6); // 6 digit di belakang koma

      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6); // 6 digit di belakang koma

      Serial.print("Speed (km/h): ");
      Serial.println(gps.speed.kmph());

      Serial.print("Course (degrees): ");
      Serial.println(gps.course.deg());

      Serial.print("Altitude (m): ");
      Serial.println(gps.altitude.meters());

      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());

      Serial.print("HDOP: ");
      Serial.println(gps.hdop.value());
    }
  }

  // Tunggu sejenak sebelum melakukan loop berikutnya
  delay(1000);
}
