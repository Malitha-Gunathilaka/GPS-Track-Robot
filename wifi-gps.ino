#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

// GPS setup
#define RXPin 18
#define TXPin 19
#define GPSBaud 9600
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// WiFi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// ESP8266 server
WiFiServer server(80);

// Variables for GPS and target coordinates
double currentLat = 0.0;
double currentLon = 0.0;
double targetLat = 37.4219983; // Default target latitude
double targetLon = -122.084;  // Default target longitude

void setup() {
  Serial.begin(115200);          // Debugging
  gpsSerial.begin(GPSBaud);      // GPS module
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Update GPS coordinates
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
      Serial.print("Current Location: ");
      Serial.print(currentLat, 6);
      Serial.print(", ");
      Serial.println(currentLon, 6);
    }
  }

  // Handle incoming client requests
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");
    String request = client.readStringUntil('\r');
    client.flush();

    // Parse GET request for target coordinates
    if (request.indexOf("GET /update") >= 0) {
      int latIndex = request.indexOf("lat=");
      int lonIndex = request.indexOf("lon=");
      if (latIndex > 0 && lonIndex > 0) {
        targetLat = request.substring(latIndex + 4, request.indexOf("&", latIndex)).toDouble();
        targetLon = request.substring(lonIndex + 4, request.indexOf(" ", lonIndex)).toDouble();
        Serial.print("Updated Target: ");
        Serial.print(targetLat, 6);
        Serial.print(", ");
        Serial.println(targetLon, 6);
      }
    }

    // Send the web page
    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    client.print("<!DOCTYPE HTML>");
    client.print("<html>");
    client.print("<h1>GPS Navigation</h1>");
    client.print("<p>Current Location: ");
    client.print(currentLat, 6);
    client.print(", ");
    client.print(currentLon, 6);
    client.print("</p>");
    client.print("<p>Target Location: ");
    client.print(targetLat, 6);
    client.print(", ");
    client.print(targetLon, 6);
    client.print("</p>");
    client.print("<form action='/update'>");
    client.print("Target Latitude: <input type='text' name='lat'><br>");
    client.print("Target Longitude: <input type='text' name='lon'><br>");
    client.print("<input type='submit' value='Update Target'>");
    client.print("</form>");
    client.print("</html>");
    client.stop();
    Serial.println("Client disconnected");
  }
}
