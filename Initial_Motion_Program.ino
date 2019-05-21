#include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#define duration 100

String Direction;
int Speed = 220; //Motor speed
//int inverted;
float Zaccel; //Acceleration from ADXL345

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

sensor_t sensor;

AF_DCMotor Lmotor(1);
AF_DCMotor Rmotor(4);


void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);

  Lmotor.setSpeed(Speed);
  Rmotor.setSpeed(Speed * 1.15909);



  if (!accel.begin()) {}
  accel.setRange(ADXL345_RANGE_16_G);

  sensors_event_t event;
  accel.getEvent(&event);
  Zaccel = (event.acceleration.z);
/*
  if (Zaccel <= 0) {
    Lmotor.setSpeed(Speed);
    Rmotor.setSpeed(Speed * 1.15909); //Tests if robot is upright or inverted
  }
  if (Zaccel >> 0) {
    //  Lmotor.setSpeed(-1 * Speed);
    //  Rmotor.setSpeed(-1 * Speed * 1.15909); //Tests if robot is upright or inverted
  }
*/
  Lmotor.run(RELEASE);
  Rmotor.run(RELEASE);
}
void loop() {



  while (Serial1.available())
  {
    delay(10);
    char c = Serial1.read();
    Direction += c;
    Serial.write(c);
  }

      sensors_event_t event;
  accel.getEvent(&event);
  Zaccel = (event.acceleration.z);
  
  if (Direction.length() > 0)
  {


    //Serial.println(Zaccel);

    //Serial1.println(Direction);
    if (Direction == ".FWD.")// Forward Function
    {
      if (Zaccel < 0) {
        Forward();
      }
      if (Zaccel > 0) {
        Reverse();
      }
    }
    if (Direction == ".STOP.")// Stop Function
    {
      Stop();
    }
    if (Direction == ".REV.")// Reverse Function
    {
      if (Zaccel < 0) {
        Reverse();
      }
      if (Zaccel > 0) {
        Forward();
      }
    }
    if (Direction == ".RIGHT.")// Pivot Right Function
    {
        Right();
    }
    if (Direction == ".LEFT.")// Pivot Left Function
    {
        Left();  
    }

    if (Direction == ".PARTY.")// Party Function (This just moves around for a few seconds)
    {
      Party();
    }
  }
  Direction = "";// Resets String
}
//}

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




