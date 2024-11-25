#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>

// GPS module pins
#define RXPin 18
#define TXPin 19
#define GPSBaud 9600

// Motor driver setup
AF_DCMotor leftMotor(1);  // Motor connected to M1
AF_DCMotor rightMotor(2); // Motor connected to M2

// Target GPS coordinates
double targetLatitude = 37.4219983;   // Replace with your target latitude
double targetLongitude = -122.084;   // Replace with your target longitude

// Tolerance for reaching the destination (in meters)
const float tolerance = 3.0;

// GPS setup
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() {
  Serial.begin(9600);               // For debugging
  gpsSerial.begin(GPSBaud);         // GPS module serial

  Serial.println("GPS Robot Initialized");
}

void loop() {
  // Check GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      double currentLatitude = gps.location.lat();
      double currentLongitude = gps.location.lng();
      Serial.print("Current Location: ");
      Serial.print(currentLatitude, 6);
      Serial.print(", ");
      Serial.println(currentLongitude, 6);

      float distanceToTarget = calculateDistance(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
      Serial.print("Distance to target: ");
      Serial.println(distanceToTarget);

      if (distanceToTarget < tolerance) {
        stopMotors();
        Serial.println("Target reached!");
      } else {
        moveTowardsTarget(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
      }
    }
  }
}

// Calculate distance using Haversine formula (in meters)
float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000; // Earth's radius in meters
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Move robot towards the target
void moveTowardsTarget(double currentLat, double currentLon, double targetLat, double targetLon) {
  double angle = calculateBearing(currentLat, currentLon, targetLat, targetLon);
  Serial.print("Bearing: ");
  Serial.println(angle);

  // Example movement logic based on angle (simplified)
  if (angle > 45 && angle < 135) {
    moveForward();
  } else if (angle >= 135 && angle <= 225) {
    turnRight();
  } else if (angle > 225 && angle < 315) {
    moveBackward();
  } else {
    turnLeft();
  }
}

// Calculate bearing between two coordinates
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double bearing = atan2(y, x);
  return degrees(bearing);
}

// Motor control functions
void moveForward() {
  leftMotor.setSpeed(200);  // Speed: 0-255
  rightMotor.setSpeed(200);
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
  Serial.println("Moving forward");
}

void moveBackward() {
  leftMotor.setSpeed(200);
  rightMotor.setSpeed(200);
  leftMotor.run(BACKWARD);
  rightMotor.run(BACKWARD);
  Serial.println("Moving backward");
}

void turnLeft() {
  leftMotor.setSpeed(150);
  rightMotor.setSpeed(200);
  leftMotor.run(BACKWARD);
  rightMotor.run(FORWARD);
  Serial.println("Turning left");
}

void turnRight() {
  leftMotor.setSpeed(200);
  rightMotor.setSpeed(150);
  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);
  Serial.println("Turning right");
}

void stopMotors() {
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
  Serial.println("Motors stopped");
}
