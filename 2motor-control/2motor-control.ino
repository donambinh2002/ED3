#include <Arduino.h>
#include <Servo.h>

// ******************************************
// Variable declaration
// ******************************************
int PotPin1 = A2;
int PotPin2 = A1;
int PotVal1;
int PotVal2;
float Angle1;
float Angle2;
unsigned long curr_time = 0, prev_time = 0, dt = 50000; // time interval in us

Servo ESC1; // create servo object to control a servo
Servo ESC2;

// ******************************************
// Function declaration
// ******************************************
float floatMap(float, float, float, float, float); // like the map function, but with float instead of integer
void SerialDataWrite();

// ******************************************
// void setup
// ******************************************
void setup()
{
  Serial.begin(9600);
  ESC1.attach(9,1000,2000); 
  ESC2.attach(10,1000,2000); 

  // analogReadResolution(12); // Only for microctrl with 12bit ADC
}
// ******************************************
// void loop
// ******************************************
void loop()
{
  // Data acquisition
  PotVal1 = analogRead(PotPin1);
  Angle1 = floatMap(PotVal1, 0, 1023, 0, 180);
  ESC1.write(Angle1);  

  PotVal2 = analogRead(PotPin2);
  Angle2 = floatMap(PotVal2, 0, 1023, 0, 180);
  ESC2.write(Angle2);  

  curr_time = micros();
  // Write on the COM each dt interval
  if (curr_time - prev_time >= dt)
  {
    prev_time += dt;
    SerialDataWrite();
  }
}
// ******************************************
// Function definition
// ******************************************
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SerialDataWrite()
{
  Serial.print(curr_time / 1000);
  Serial.print("     ");
  Serial.print(PotVal1);
  Serial.print(",");
  Serial.print(Angle1);
  Serial.print("-----------");
  Serial.print(PotVal2);
  Serial.print(",");
  Serial.print(Angle2);
  Serial.println("");
}
