#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>

// GPS module pins
#define RXPin 18
#define TXPin 19
#define GPSBaud 9600

// Motor driver setup
AF_DCMotor leftMotorFront(1);  // Motor connected to M1
AF_DCMotor rightMotorFront(2); // Motor connected to M2
AF_DCMotor leftMotorRear(3);   // Motor connected to M3
AF_DCMotor rightMotorRear(4);  // Motor connected to M4

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
  leftMotorFront.setSpeed(200);  // Speed: 0-255
  rightMotorFront.setSpeed(200);
  leftMotorRear.setSpeed(200);
  rightMotorRear.setSpeed(200);
  leftMotorFront.run(FORWARD);
  rightMotorFront.run(FORWARD);
  leftMotorRear.run(FORWARD);
  rightMotorRear.run(FORWARD);
  Serial.println("Moving forward");
}

void moveBackward() {
  leftMotorFront.setSpeed(200);
  rightMotorFront.setSpeed(200);
  leftMotorRear.setSpeed(200);
  rightMotorRear.setSpeed(200);
  leftMotorFront.run(BACKWARD);
  rightMotorFront.run(BACKWARD);
  leftMotorRear.run(BACKWARD);
  rightMotorRear.run(BACKWARD);
  Serial.println("Moving backward");
}

void turnLeft() {
  leftMotorFront.setSpeed(150);
  rightMotorFront.setSpeed(200);
  leftMotorRear.setSpeed(150);
  rightMotorRear.setSpeed(200);
  leftMotorFront.run(BACKWARD);
  rightMotorFront.run(FORWARD);
  leftMotorRear.run(BACKWARD);
  rightMotorRear.run(FORWARD);
  Serial.println("Turning left");
}

void turnRight() {
  leftMotorFront.setSpeed(200);
  rightMotorFront.setSpeed(150);
  leftMotorRear.setSpeed(200);
  rightMotorRear.setSpeed(150);
  leftMotorFront.run(FORWARD);
  rightMotorFront.run(BACKWARD);
  leftMotorRear.run(FORWARD);
  rightMotorRear.run(BACKWARD);
  Serial.println("Turning right");
}

void stopMotors() {
  leftMotorFront.setSpeed(0);
  rightMotorFront.setSpeed(0);
  leftMotorRear.setSpeed(0);
  rightMotorRear.setSpeed(0);
  leftMotorFront.run(RELEASE);
  rightMotorFront.run(RELEASE);
  leftMotorRear.run(RELEASE);
  rightMotorRear.run(RELEASE);
  Serial.println("Motors stopped");
}