// Teensy 4.1 and Victoria
// Copyright (C) 2026
// fb80a38a-b885-4b09-9b2e-14254ae0a193
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.

// Arduino 2.3.7 IDE    https://www.arduino.cc/en/software
// Teensyduino 1.59     https://www.pjrc.com/teensy/td_download.html
// My libraries         https://github.com/pwesson/roboticboat
//                      https://github.com/pwesson/compass
//                      https://github.com/pwesson/gps

// Memory Usage on Teensy 4.1:
//   FLASH: code:103676, data:11448, headers:8776   free for files:8002564
//   RAM1: variables:20096, code:100584, padding:30488   free for local variables:373120
//   RAM2: variables:12416  free for malloc/new:511872

// Teensy 4.1 Autonomous Sailboat Controller
// Modernised + Non‑Blocking Rewrite
// Paul @ roboticboat.uk  — Refactored by Copilot

#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <CompassCMPS14.h>
#include <ParallaxSIM33EAU.h>
#include <Rudder.h>
#include <WeightedLeastSquares.h>
#include <BNO086.h>
#include <Navigation.h>

// ======================================================
// Constants
// ======================================================
#define RECENT_HISTORY 60
#define RC_RUDDER_PIN 3
#define RC_SAIL_PIN 4
#define RC_HUMAN_ROBOT_PIN 5
#define RUDDER_PIN 7
#define SAIL_PIN   8
#define LED_PIN    13

// ======================================================
// Timing
// ======================================================
elapsedMillis timer500;     // 500 ms tasks
elapsedMillis servoTimer;   // 20 ms servo updates

// ======================================================
// RC PWM Input (non‑blocking, interrupt‑driven)
// ======================================================
struct RCChannel {
    volatile uint32_t riseTime = 0;
    volatile uint32_t pulseWidth = 1500;
};

RCChannel rc1, rc2, rc3;

// ISR for channel 1 (pin 3) - Rudder control
void rc1_isr() {
    if (digitalReadFast(RC_RUDDER_PIN)) {
        rc1.riseTime = micros();
    } else {
        rc1.pulseWidth = micros() - rc1.riseTime;
    }
}

// ISR for channel 2 (pin 4) - Sail winch control
void rc2_isr() {
    if (digitalReadFast(RC_SAIL_PIN)) {
        rc2.riseTime = micros();
    } else {
        rc2.pulseWidth = micros() - rc2.riseTime;
    }
}

// ISR for channel 3 (pin 5) - Switch Human/Robot control
void rc3_isr() {
    if (digitalReadFast(RC_HUMAN_ROBOT_PIN)) {
        rc3.riseTime = micros();
    } else {
        rc3.pulseWidth = micros() - rc3.riseTime;
    }
}

// Initial positions
int ch1 = 90, ch2 = 90, ch3 = 90;

void readRC() {

    ch1 = constrain(map(rc1.pulseWidth, 1000, 2000, 45, 135), 45, 135);
    ch2 = constrain(map(rc2.pulseWidth, 1000, 2000, 45, 135), 45, 135);
    ch3 = constrain(map(rc3.pulseWidth, 1000, 2000, 45, 135), 45, 135);
}

// ======================================================
// Servos
// ======================================================
Servo servoRudder;
Servo servoSail;

// ======================================================
// Sensors
// ======================================================
CompassCMPS14 cmps14;
BNO086 bno086;
ParallaxSIM33EAU gps;
Navigation navigate;

// IMU history
float yawHist[RECENT_HISTORY] = {0};
float latHist[RECENT_HISTORY] = {0};
float lonHist[RECENT_HISTORY] = {0};
int histPtr = 0;
float yawSum = 0;

// Boat heading
float BoatHeading = 0;
float oldBoatHeading = 0;
float gpsBearing = 0;

// Rudder ML
Rudder myrudder;

// SD logging
File dataFile;
char sdbuffer[500];
long dataFilesize = 0;
long dataFilesize_old = 0;

// Misc
int outputV1 = 0;
float bno086_roll = 0;
float bno086_pitch = 0;
float bno086_yaw = 0;

// ======================================================
// Utility
// ======================================================
inline float smooth(float old, float input, float alpha) {
    return old * alpha + input * (1 - alpha);
}

// ======================================================
// IMU
// ======================================================
void readIMU() {

    bno086.receivePacket();
    //if (!bno086.receivePacket()) return;

    auto &q = bno086.quantGameRotationVec;

    if (q.status == 3 && q.real > 0.01f && q.real <= 1.0f) {
        bno086.Quanternion2Euler(q);

        // Store local copy
        bno086_roll = bno086.euler.roll;
        bno086_pitch = bno086.euler.pitch;
        bno086_yaw = bno086.euler.yaw;

    }
}

// ======================================================
// Compass
// ======================================================
void readCompass() {

    cmps14.ReadCompass();

    float heading = cmps14.bearing;
    if (heading != 0) BoatHeading = heading;

    cmps14.ReadGyro();
    cmps14.ReadAccelerator();
    cmps14.getCalibration();

    myrudder.GyroUpdateMean(BoatHeading, cmps14.gyroZ);
    myrudder.AddGyroReading(ch1, cmps14.gyroZ);
}

// ======================================================
// GPS
// ======================================================
void readGPS() {

    while (Serial1.available()) {

        gps.listen();
    }
}

// ======================================================
// GPS + Navigation
// ======================================================
void updateNavigation() {

    if (gps.gpsstatus != 'A') return;

    //yawSum -= yawHist[histPtr];
    //yawSum += bno086.euler.yaw;

    //yawHist[histPtr] = bno086.euler.yaw;
    latHist[histPtr] = gps.latitude;
    lonHist[histPtr] = gps.longitude;

    int next = (histPtr + 1) % RECENT_HISTORY;

    coordinate start = { latHist[next], lonHist[next] };
    coordinate end   = { gps.latitude, gps.longitude };

    gpsBearing = navigate.bearing(start, end);

    histPtr = next;
}

// ======================================================
// Robot/Human Control
// ======================================================
bool IsHuman = true;

void updateControlMode() {

    bool robotRequested = (ch3 > 100);

    if (robotRequested && IsHuman) {
        // Transition: Human → Robot
        // myrudder.NewLine(gps.latitude, gps.longitude, BoatHeading);
    }

    if (!robotRequested && !IsHuman) {
        // Transition: Robot → Human
        myrudder.ClearLine();
    }

    IsHuman = !robotRequested;
}

// ======================================================
// Servo Output (rate‑limited)
// ======================================================
void updateServos() {

    if (servoTimer < 20) return;
    servoTimer = 0;

    if (IsHuman) {
        servoRudder.write(ch1);
        servoSail.write(ch2);
    } else {
        servoRudder.write(ch1);  // ML can replace this later
        servoSail.write(ch2);
    }
}

// ======================================================
// Logging (500 ms)
// ======================================================
void updateLogging() {

    if (timer500 < 500) return;
    timer500 = 0;

    if (dataFilesize != dataFilesize_old) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        dataFilesize_old = dataFilesize;
    }

    float diff = BoatHeading - oldBoatHeading;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    if (!IsHuman) {
        myrudder.AddReading(millis(), ch1, diff);
    }

    oldBoatHeading = BoatHeading;

    // Update the User
    sprintf(sdbuffer, "Victoria44,$TMR,%lu,$RC,%d,%d,%d,$CMP,%.0f,%d,%d,%c%c%c%c%c%c%c%c,$ACC,%.3f,%.3f,%.3f,$GYR,%.3f,%.3f,%.3f,$GPS,%.0f,%.0f,%.8f,%c,%.8f,%c,%.1f,%d,%d,%.2f,%.2f,%c,%.0f,$HALL,%d,$BNO,%.0f,%.0f,%.0f,%.1f,%.1f", 
                      millis(),ch1,ch2,ch3,
                      BoatHeading,cmps14.pitch,cmps14.roll,BYTE_TO_BINARY(cmps14.calibration),
                      cmps14.accelX,cmps14.accelY,cmps14.accelZ,
                      cmps14.gyroX,cmps14.gyroY,cmps14.gyroZ - myrudder.gyrozmean,
                      gps.gpsdate,gps.gpstime,gps.latitude,gps.latNS,gps.longitude,gps.lonEW,gps.altitude,gps.fixquality,gps.numsatelites,gps.gpsknots,gps.gpstrack,gps.gpsstatus,gpsBearing,
                      outputV1,
                      bno086_roll, bno086_pitch, bno086_yaw, 0.0, 0.0);

    if (Serial) {
        Serial.printf("%s\n", sdbuffer);
        Serial.flush();
    }

    if (dataFile) {
        dataFile.printf("%s\n", sdbuffer);
        dataFile.flush();
        dataFilesize = dataFile.size();
    }
}

// ======================================================
// Hall Sensor
// ======================================================

void readHall()
{
  // Take 10 readings
  outputV1 = 0;
  
  for(int i = 0; i < 10; i++)
  {
      outputV1 += analogRead(A0);
      delay(1);
  }

}

// ======================================================
// SD card
// ======================================================
void InitialiseSDcard(int testseconds)
{
  Serial.println("Initializing SD card...");

  SD.begin(BUILTIN_SDCARD);

  delay(100);

  if (ReadWriteTest()){
    Serial.println("OK");
    digitalWrite(LED_PIN, HIGH);
    delay(testseconds * 1000);
    digitalWrite(LED_PIN, LOW);
  }
  else
  {
    Serial.println("ERROR");
    for (int i=1;i<=testseconds;i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }  
}

bool ReadWriteTest()
{
  File myFile;
  char filename[] = "testfile.txt";
  char writestring[] = "abcdefghijklmnopqrstuvwxyz1234567890";
  char readstring[40];

  // First remove the file is it already exists
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  // Open file to write
  myFile = SD.open(filename, FILE_WRITE);
  
  // If okay, write to the file
  if (myFile) {
    Serial.print("Writing to file...  ");
    myFile.print(writestring);
    myFile.close();
    Serial.print('[');
    Serial.print(writestring);
    Serial.println("] done.");
  } 
  else 
  {
    // Error writing to the file
    Serial.println("error opening testfile.txt");
  }
  
  // Open file to read. Which is the default option
  myFile = SD.open(filename, FILE_READ);
  if (myFile) {
    Serial.print("Reading from file...");
    int n = 0;
    while (myFile.available()) {
      if (n<39)
      {
        readstring[n] = myFile.read();
        readstring[n+1] = '\0';
      }
      n=n+1;
    }
    myFile.close();
    Serial.print('[');
    Serial.print(readstring);
    Serial.println("] done.");
  } 
  else 
  {
    // Error reading from the file
    Serial.println("error opening testfile.txt");
  }

  // Return true if the two char arrays are equal
  if (strcmp(writestring, readstring) == 0){
    return true;
  }
  return false;
}


// ======================================================
// Setup
// ======================================================
void setup() {

    Serial.begin(115200);
    delay(100);

    // Teensy 4.1 onboard LED light - feedback to user
    pinMode(LED_PIN, OUTPUT);

    // Hall sensor
    pinMode(A0, INPUT);

    // RC receiver channels
    pinMode(RC_RUDDER_PIN, INPUT);
    pinMode(RC_SAIL_PIN, INPUT);
    pinMode(RC_HUMAN_ROBOT_PIN, INPUT);

    // RC interrupts
    attachInterrupt(RC_RUDDER_PIN, rc1_isr, CHANGE);
    attachInterrupt(RC_SAIL_PIN, rc2_isr, CHANGE);
    attachInterrupt(RC_HUMAN_ROBOT_PIN, rc3_isr, CHANGE);

    // Servos
    servoRudder.attach(RUDDER_PIN);
    servoSail.attach(SAIL_PIN);

    // GPS
    Serial1.begin(9600);
    gps.setSerial(&Serial1);
    gps.SelectSentences();

    // I2C
    Wire.begin();
    Wire.setClock(100000);
    delay(100);

    // Compass
    cmps14.getCalibration();

    // IMU
    bno086.SetupBMO086();

    // SD
    InitialiseSDcard(1);

    //SD.begin(BUILTIN_SDCARD);
    dataFile = SD.open("datalog.txt", FILE_WRITE);
}


// ======================================================
// Main Loop
// ======================================================
void loop() {

    readRC();
    readIMU();
    readCompass();
    readGPS();
    readHall(); 
    updateNavigation();
    updateControlMode();
    updateServos();
    updateLogging();

}