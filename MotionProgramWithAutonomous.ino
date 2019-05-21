#include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#define duration 100 //Time to drive in any direction (except in autonomous modes) before looping again
#define TurnDuration 200 //Time to turn away from obstacles in avoidance mode

#include <NewPing.h>

#define SONAR_NUM 5      // Number of sensors.
#define MAX_DISTANCE 30 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(A1, A1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A2, MAX_DISTANCE),
  NewPing(A3, A3, MAX_DISTANCE),
  NewPing(A0, A0, MAX_DISTANCE),
  NewPing(A4, A4, MAX_DISTANCE)
};

String Direction;
String DirectionNew;
int Speed = 220; //Motor speed
unsigned long SensorValues[5];//Array that will record distances for obstacle avoidance to use
unsigned long ReadSensorValues[5];
int MinimumDistance = 50;//this value will be used to set the minimum distance (cm) an obstace is allowed to be near any given sensor
int RandomInt; //Random value for choosing which way to turn
float Zaccel; //Acceleration from ADXL345

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

sensor_t sensor;

AF_DCMotor Lmotor(1);
AF_DCMotor Rmotor(4);

void setup() {
  Serial1.begin(9600);
  Serial.begin(115200);

  Lmotor.setSpeed(Speed);//Defines motor speeds while taking into effect that my motors do not drive the same
  Rmotor.setSpeed(Speed * 1.15909);//Defines motor speeds while taking into effect that my motors do not drive the same

  if (!accel.begin()) {}
  accel.setRange(ADXL345_RANGE_16_G);

  sensors_event_t event;
  accel.getEvent(&event);
  Zaccel = (event.acceleration.z);

  Lmotor.run(RELEASE);
  Rmotor.run(RELEASE);
}

void loop() {

  SensorRead();

  DirectionRead();


  //if (DirectionNew != Direction) {
  //DirectionNew = "";// Resets String
  // DirectionNew = Direction;
  //  }


  sensors_event_t event;
  accel.getEvent(&event);
  Zaccel = (event.acceleration.z);

  if (Direction.length() > 0)
  {
    SensorRead();

    if (Direction == ".AVOID.")// Autonomous Function to Avoid
    {
      Serial.print("avd");
      SensorAvoid();
    }
    else if (Direction == ".Follow.")// Autonomous Function to Avoid
    {
      SensorFollow();
    }
    else if (Direction == ".FWD.")// Forward Function
    {
      if (Zaccel < 0) {
        Forward();
      }
      else if (Zaccel > 0) {
        Reverse();
      }
    }
    else if (Direction == ".STOP.")// Stop Function
    {
      Stop();
    }
    else if (Direction == ".REV.")// Reverse Function
    {
      if (Zaccel < 0) {
        Reverse();
      }
      else if (Zaccel > 0) {
        Forward();
      }
    }
    else if (Direction == ".RIGHT.")// Pivot Right Function
    {
      Right();
    }
    else if (Direction == ".LEFT.")// Pivot Left Function
    {
      Left();
    }

    else if (Direction == ".PARTY.")// Party Function (This just moves around for a few seconds)
    {
      Party();
    }
    else {
      Stop();
    }
  }
  Direction = "";// Resets String

}

void Forward() {
  Lmotor.run(FORWARD);
  Rmotor.run(FORWARD);
  delay(duration);
}

void Reverse() {
  Lmotor.run(BACKWARD);
  Rmotor.run(BACKWARD);
  delay(duration);
}

void Left() {
  Lmotor.run(BACKWARD);
  Rmotor.run(FORWARD);
  delay(duration);
}

void Right() {
  Lmotor.run(FORWARD);
  Rmotor.run(BACKWARD);
  delay(duration);
}

void Stop() {
  Lmotor.run(RELEASE);
  Rmotor.run(RELEASE);
  delay(duration);
}

void SensorRead() {
  //SensorValues[] = {0, 0, 0, 0, 0};
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    SensorValues[i] = 0;
    delay(40); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    ReadSensorValues[i] = (sonar[i].ping_cm());
    Serial.print(ReadSensorValues[i]);
    Serial.print(",");
    if (ReadSensorValues[i] <= MinimumDistance && ReadSensorValues[i] != 0) {
      SensorValues[i] = 1;
      // Serial.print("too close");
    }
    else {
      SensorValues[i] = 0;
    }
  }
  PrintSensorValues();
}

void PrintSensorValues() {
  Serial.println();
  Serial.print(SensorValues[0]);
  Serial.print(SensorValues[1]);
  Serial.print(SensorValues[2]);
  Serial.print(SensorValues[3]);
  Serial.println(SensorValues[4]);
}

void AvoidRandom() {
  Serial.println("Random");
  RandomInt = random(0, 2);//creates random value of 1 or 0 to choose to go left or right
  if (RandomInt == 1) {
    AvoidLeft();
  }
  else {
    AvoidRight();
  }
}

void AvoidLeft() {
  Serial.println("Left");
  Stop();
  Right();
  delay(TurnDuration);
  Stop();
}

void AvoidRight() {
  Serial.println("Right");
  Stop();
  Left();
  delay(TurnDuration);
  Stop();
}

void  SensorFollow() {
  //not programmed yet
}
void SensorAvoid() {
  while (1) {
    Serial.print("  avoid loop");
    DirectionRead();
    //  while (Serial1.available())
    //   {
    //  Serial.print("  avoid While");
    //  delay(10);
    //  char c = Serial1.read();
    //  DirectionNew += c;
    //  Serial.write(c);
    //    }
    //while (DirectionNew == ".AVOID.") {
    if (Direction == ".STOP." || Direction == ".FWD." || Direction == ".REV." || Direction == ".PARTY." || Direction == ".LEFT." || Direction == ".RIGHT." || Direction == "" || Direction == ".") {
      break;
    }
    else if (Direction == ".AVOID.") {
      DirectionRead();
      //    while (Serial1.available())
      //   {
      // // Serial.print("  avoid While");
      //  delay(10);
      //  char c = Serial1.read();
      //  DirectionNew += c;
      // // Serial.write(c);
      //    }


      Serial.print("  Dir While");

      SensorRead();

      if (SensorValues[0] == 0 && SensorValues[1] == 0 && SensorValues[2] == 1 && SensorValues[3] == 0 && SensorValues[4] == 0) {//00100
        AvoidRandom();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 1 && SensorValues[2] == 1 && SensorValues[3] == 1 && SensorValues[4] == 0) {//01110
        AvoidRandom();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 0 && SensorValues[2] == 0 && SensorValues[3] == 0 && SensorValues[4] == 0) {//00000
        Forward();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 1 && SensorValues[2] == 0 && SensorValues[3] == 0 && SensorValues[4] == 0) {//01000
        AvoidLeft();
      }
      else if (SensorValues[0] == 1 && SensorValues[1] == 1 && SensorValues[2] == 0 && SensorValues[3] == 0 && SensorValues[4] == 0) {//11000
        AvoidLeft();
      }
      else if (SensorValues[0] == 1 && SensorValues[1] == 1 && SensorValues[2] == 1 && SensorValues[3] == 0 && SensorValues[4] == 0) {//11100
        AvoidLeft();
      }
      else if (SensorValues[0] == 1 && SensorValues[1] == 0 && SensorValues[2] == 0 && SensorValues[3] == 0 && SensorValues[4] == 0) {//10000
        AvoidLeft();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 0 && SensorValues[2] == 0 && SensorValues[3] == 1 && SensorValues[4] == 0) {//00010
        AvoidRight();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 0 && SensorValues[2] == 0 && SensorValues[3] == 1 && SensorValues[4] == 1) {//00011
        AvoidRight();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 0 && SensorValues[2] == 1 && SensorValues[3] == 1 && SensorValues[4] == 0) {//00110
        AvoidRight();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 0 && SensorValues[2] == 1 && SensorValues[3] == 1 && SensorValues[4] == 1) {//00111
        AvoidRight();
      }
      else if (SensorValues[0] == 0 && SensorValues[1] == 0 && SensorValues[2] == 0 && SensorValues[3] == 0 && SensorValues[4] == 1) {//00001
        AvoidRight();
      }
      else if (SensorValues[0] == 1 && SensorValues[1] == 1 && SensorValues[2] == 1 && SensorValues[3] == 1 && SensorValues[4] == 1) {//11111
        AvoidRandom();
      }
      else {//?????
        AvoidRandom();
      }
      // if (
    }
    //  else {
    //    break;
    //  }
  }
}

void DirectionRead() {
  while (Serial1.available())
  {
    delay(10);
    char c = Serial1.read();
    Direction += c;
    Serial.write(c);
  }
}

void Party() {
  for (int i = 0 ; i < 4; i++) {
    Lmotor.run(BACKWARD);
    Rmotor.run(BACKWARD);
    delay(300);
    Lmotor.run(FORWARD);
    Rmotor.run(FORWARD);
    delay(400);
    Lmotor.run(BACKWARD);
    Rmotor.run(FORWARD);
    delay(500);
    Rmotor.run(BACKWARD);
    Lmotor.run(FORWARD);
    delay(300);
    Lmotor.run(BACKWARD);
    Rmotor.run(BACKWARD);
    delay(600);
    Lmotor.run(FORWARD);
    Rmotor.run(FORWARD);
    delay(200);
    Lmotor.run(BACKWARD);
    Rmotor.run(FORWARD);
    delay(500);
    Rmotor.run(BACKWARD);
    Lmotor.run(FORWARD);
    delay(300);
    Rmotor.run(RELEASE);
    Lmotor.run(RELEASE);
    delay(100);
  }
}




