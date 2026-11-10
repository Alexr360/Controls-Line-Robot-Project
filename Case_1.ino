#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);

MeUltrasonicSensor ultraSensor(PORT_6);
MeLineFollower lineFinder(PORT_5);

int leftMotor = 200;
int rightMotor = -200;

float turnRatio = 0.5;
float slowRatio = 0.5;

int stopDistance = 10; //Distance in cm from front tread4


// Last known direction:
// -1 = Left
//  1 = Right
//  0 = Unknown

int lastDirection = 0;

// Time when the line was first lost
unsigned long lineLostTime = 0;

// Maximum recovery duration (milliseconds)
const unsigned long recoveryDuration = 5000;

// Whether robot is currently searching
bool searchingForLine = false;

// ==========================================
// MOTOR CONTROL FUNCTIONS
// ==========================================

// Stop all motors
void stopMotors() {
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
}

// Drive forward
void moveForward() {
  motor2.run(rightMotor);
  motor4.run(leftMotor);
}

// Drive backwards
void moveBackwards() {
  motor2.run(-rightMotor);
  motor4.run(-leftMotor);
}

// Turn left
void turnLeft() {
  motor2.run(rightMotor);
  motor4.run(-leftMotor*turnRatio);
}

// Turn right
void turnRight() {
  motor2.run(-rightMotor*turnRatio);
  motor4.run(leftMotor);
}

// Turn right
void moveBackSlow() {
  motor2.run(-rightMotor*slowRatio);
  motor4.run(-leftMotor*slowRatio);
}

// SETUP

void setup() {
  Serial.begin(9600);

  stopMotors();

  Serial.println("Line Following Robot Initialized");
  Serial.println("Recovery timeout: 2500 ms");
}

// MAIN CONTROL LOOP

void loop() {
  // Read line follower sensors
  int sensorState = lineFinder.readSensors();
  
  int distance = ultraSensor.distanceCm() - 4;

  if (distance >= 5) {
    // CASE 1: BOTH SENSORS DETECT BLACK
    if (sensorState == S1_IN_S2_IN) {
      Serial.print("Both sensors detect black - Forward");
      // Stop completely
      moveForward();
      // Reset recovery state
      searchingForLine = false;
      // Clear last known direction
      lastDirection = 0;
    }

    // CASE 2: SENSOR 2 OUTSIDE BLACK LINE
    else if (sensorState == S1_IN_S2_OUT) {
      Serial.print("Sensor 2 outside - Turning Left");
      // Turn left
      turnLeft();
      // Remember last direction
      lastDirection = -1;
      // Reset recovery mode
      searchingForLine = false;
    }

    // CASE 3: SENSOR 1 OUTSIDE BLACK LINE
    else if (sensorState == S1_OUT_S2_IN) {
      Serial.print("Sensor 1 outside - Turning Right");
      // Turn right
      turnRight();
      // Remember last direction
      lastDirection = 1;
      // Reset recovery mode
      searchingForLine = false;
    }

    // CASE 4: BOTH SENSORS LOSE THE LINE
    else if (sensorState == S1_OUT_S2_OUT) {

      // Start the recovery timer only once
      if (!searchingForLine) {
        lineLostTime = millis();
        searchingForLine = true;
        Serial.print("Line Lost - Starting Recovery");
      }

      // Calculate elapsed recovery time
      unsigned long elapsedTime = millis() - lineLostTime;

      // RECOVERY ACTIVE (LESS THAN 2.5 SEC)

      if (elapsedTime < recoveryDuration / 2) {

        // Last detected line was to the left
        if (lastDirection == -1) {
          Serial.print("Searching Left");
          turnLeft();
        }

        // Last detected line was to the right
        else if (lastDirection == 1) {
          Serial.print("Searching Right");
          turnRight();
        }

        // No known previous direction
        else {
          if (elapsedTime < recoveryDuration) {
            moveBackSlow();
          }
          else {
            Serial.print("No Known Direction - STOP");
            stopMotors();
          }
        }
      }
      else if (elapsedTime < (recoveryDuration*1.5)) {
        turnRight();
      }
      // RECOVERY TIMEOUT (2.5 SEC REACHED)
      else {
        Serial.print("Recovery Timeout - STOP");
        stopMotors();
      }
    }
    Serial.print(",  Distance : ");
  }
  else {
    Serial.print("Obstacle Detected, Distance : ");
    stopMotors();
  }

  Serial.print(distance);
  Serial.println(" cm");
}
