#include <Arduino.h>
#include <Wire.h>

// ******************************************
// General information
// ******************************************
// IMU testing. No library used, except Wire to use the I2C

// ******************************************
// Variable declaration
// ******************************************
// IMU parameters
int16_t Acc_raw_X, Acc_raw_Y, Acc_raw_Z;
int16_t Gyro_raw_X, Gyro_raw_Y, Gyro_raw_Z;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float GyroX_calib, GyroY_calib, GyroZ_calib;
float AccX_calib, AccY_calib, AccZ_calib;
// long unsigned elapsedTime, time, timePrev;
long unsigned prev_time_mpu, dt_mpu;
float rad_to_deg = 180 / 3.141592654;
float Acc_angle[3];
float Gyro_angle[3] = {0, 0, 0};
float Total_angle[3] = {0, 0, 0};
int i;

// Serial configuration parameters
unsigned long curr_time = 0, prev_time_com = 0, dt_com = 50000; // time interval in us

// ******************************************
// Function declaration
// ******************************************
void dataReadyISR();
void SerialDataWrite();
void gyro_signals();
void acc_signals();
float floatMap(float, float, float, float, float);

// ******************************************
// void setup
// ******************************************
void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  // MPU preparation and calibration
  Wire.setClock(400000); // set wire clock to the max (400kHz)
  Wire.begin();          // Start the wire comunication
  delay(100);
  Wire.beginTransmission(0x68); // Start the communication with the MPU6050
  Wire.write(0x6B);             // Start the gyro in power mode
  Wire.write(0x00);             // Set the requested starting register
  Wire.endTransmission();       // End the transmission
  // MPU6050 Gyro Calibration
  // Serial.println("Begin MPU Gyro Calibration");
  for (i = 0; i < 2000; i++)
  {
    gyro_signals();
    GyroX_calib += GyroX;
    GyroY_calib += GyroY;
    GyroZ_calib += GyroZ;
  }
  GyroX_calib /= 2000;
  GyroY_calib /= 2000;
  GyroZ_calib /= 2000;
  // MPU6050 Acc Calibration
  // Serial.println("Begin MPU Acc  Calibration");
  for (i = 0; i < 2000; i++)
  {
    acc_signals();
    AccX_calib += AccX;
    AccY_calib += AccY;
    AccZ_calib += AccZ;
  }
  // For the calibration of the accelerometer, check what is the direction of the MPU. For me g is in the negative X-axis
  // Because the gravitational acceleration should not be part of the calibration
  AccX_calib = AccX_calib / 2000 - 1;
  AccY_calib = AccY_calib / 2000;
  AccZ_calib = AccZ_calib / 2000;

  // Serial.println("MPU calibrated. Press any key to start the main program.");
  // while (!Serial.available())
    ;
  // Serial.read();
}

void loop()
{
  curr_time = micros();
  gyro_signals(); // Get gyro data GyroX, GyroY, GyroZ
  acc_signals();  // Get acc data AccX, AccY, AccZ

  GyroX -= GyroX_calib;
  GyroY -= GyroY_calib;
  GyroZ -= GyroZ_calib;
  AccX -= AccX_calib;
  AccY -= AccY_calib;
  AccZ -= AccZ_calib;

  dt_mpu = (curr_time - prev_time_mpu);
  prev_time_mpu = curr_time;

  Acc_angle[1] = atan2(AccZ, sqrt(pow(AccY, 2) + pow(AccX, 2))) * rad_to_deg;
  Acc_angle[2] = atan2(-AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * rad_to_deg;

  Gyro_angle[0] += GyroX * dt_mpu / 1000000;
  Gyro_angle[1] += GyroY * dt_mpu / 1000000;
  Gyro_angle[2] += GyroZ * dt_mpu / 1000000;

  Total_angle[0] = Gyro_angle[0];
  Total_angle[1] = .98 * (Total_angle[1] + GyroY * dt_mpu / 1000000) + .02 * Acc_angle[1];
  Total_angle[2] = .98 * (Total_angle[2] + GyroX * dt_mpu / 1000000) + .02 * Acc_angle[2];

  // Serial write
  curr_time = micros();
  if (curr_time - prev_time_com >= dt_com)
  {
    prev_time_com += dt_com;
    SerialDataWrite();
  }
}

// ******************************************
// Function definitions
// ******************************************
void SerialDataWrite()
{
  Serial.print(curr_time / 1000);
  Serial.print(",");
  Serial.print(Total_angle[1]);
  Serial.print(",");
  Serial.print(Total_angle[2]);
  Serial.println("");
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68); // Start the communication with the MPU6050
  Wire.write(0x1A);             // Enable Low-pass filter
  Wire.write(0x05);
  Wire.endTransmission(); // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Change Gyro sensitivity
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68); // Access register storing Gyro data
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // Read the Gyro data
  Gyro_raw_X = Wire.read() << 8 | Wire.read();
  Gyro_raw_Y = Wire.read() << 8 | Wire.read();
  Gyro_raw_Z = Wire.read() << 8 | Wire.read();
  GyroX = (float)Gyro_raw_X / 65.5;
  GyroY = (float)Gyro_raw_Y / 65.5;
  GyroZ = (float)Gyro_raw_Z / 65.5;
}

void acc_signals(void)
{
  Wire.beginTransmission(0x68); // Start the communication with the MPU6050
  Wire.write(0x1A);             // Enable Low-pass filter
  Wire.write(0x05);             // Enable Low-pass filter - 10Hz-ish
  Wire.endTransmission();       // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Access register storing Acc data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // Read the Acc data
  Acc_raw_X = Wire.read() << 8 | Wire.read();
  Acc_raw_Y = Wire.read() << 8 | Wire.read();
  Acc_raw_Z = Wire.read() << 8 | Wire.read();
  AccX = (float)Acc_raw_X / 16384.0;
  AccY = (float)Acc_raw_Y / 16384.0;
  AccZ = (float)Acc_raw_Z / 16384.0;
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
