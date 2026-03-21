#include <Wire.h> // Enables I2C comms
#include <MPU6050_light.h> //Enables IMU interfacing

// Creates IMU class with a Wire class from Wire.h
MPU6050 mpu(Wire);

unsigned long previousTime = 0;
float totalAngleZ = 0.0;   // Total rotation (degrees)

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C


  byte status = mpu.begin(); // Initialize IMU class (created on line 5)
  if (status != 0) {
    Serial.print("MPU6050 init failed: ");
    Serial.println(status);
    while (1);
  }

  Serial.println("Calibrating gyro... keep sensor still!");
  delay(2000);

  mpu.calcGyroOffsets(); // Auto calibration

  previousTime = millis();
}

void loop() {
  mpu.update(); // Update Gyro data

  // Time calculation
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Gyro Z in degrees/sec
  float gyroZ = mpu.getGyroZ(); // Get updated data from the Gyro

  // Integrate to get angle
  totalAngleZ += gyroZ * dt;

  // Print results
  Serial.print("Gyro Z (deg/s): ");
  Serial.print(gyroZ);
  Serial.print(" | Total Angle Z: ");
  Serial.println(totalAngleZ);

  delay(10); // ~100 Hz sample rate
}