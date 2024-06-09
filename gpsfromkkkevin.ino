#include <NeoSWSerial.h>

// Define software serial pins for Neo-M8N communication
NeoSWSerial gpsSerial(0, 1);

// Define variables to store extracted data
char date[11];
char time[11];
double latitude = 0.0, longitude = 0.0;

void setup() {
  // Set baud rate for serial communication (default for Neo-M8N)
  gpsSerial.begin(9600);

  // Open serial monitor for data display
  Serial.begin(9600);
  Serial.println("Starting GPS Tracker...");
}

void loop() {
  // Check if data is available from Neo-M8N
  if (gpsSerial.available()) {
    char data = gpsSerial.read();

    // Look for the beginning of a valid NMEA sentence
    if (data == '$') {
      // Initialize variables for sentence parsing
      char sentence[100];
      int index = 0;

      // Read characters until the end of the sentence is reached
      while (gpsSerial.available()) {
        data = gpsSerial.read();
        sentence[index++] = data;
        if (data == '\n' || index >= sizeof(sentence) - 1) {
          break;
        }
      }
      sentence[index] = '\0';  // Add null terminator for string manipulation

      // Check for specific NMEA sentences (modify as needed)
      if (strstr(sentence, "$GPRMC") != NULL) {  // Check for Recommended Minimum Navigation Information sentence
        // Parse the sentence to extract data
        parseGPRMC(sentence);
      }
    }
  }

  // Print extracted data to serial monitor
  Serial.print("Date: ");
  Serial.println(date);
  Serial.print("Time: ");
  Serial.println(time);
  Serial.print("Latitude: ");
  Serial.println(latitude, 6);  // Print with 6 decimal places for precision
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);

  // Add a delay to avoid overwhelming the serial monitor
  delay(1000);
}

// Function to parse the GPRMC sentence and extract data
void parseGPRMC(char* sentence) {
  // Split the sentence into comma-separated fields
  char* fields[14];
  char* field = strtok(sentence, ",");
  int i = 0;
  while (field != NULL && i < 14) {
    fields[i++] = field;
    field = strtok(NULL, ",");
  }

  // Check if the data is valid (field 2)
  if (fields[2][0] != 'A') {  // 'A' means data is valid
    Serial.println("Invalid data");
    return;
  }

  // Extract date (field 9)
  strncpy(date, fields[9], sizeof(date) - 1);
  date[sizeof(date) - 1] = '\0';

  // Extract time (field 1)
  strncpy(time, fields[1], sizeof(time) - 1);
  time[sizeof(time) - 1] = '\0';

  // Extract latitude (field 3) and convert to decimal degrees
  latitude = convertToDecimalDegrees(fields[3], fields[4][0]);

  // Extract longitude (field 5) and convert to decimal degrees
  longitude = convertToDecimalDegrees(fields[5], fields[6][0]);
}

// Function to convert NMEA format to decimal degrees
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