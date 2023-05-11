#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v1.h>

const bool DEBUG = false;
const int PITCH_SERVO_PIN = 15; // D8 is GPIO 15
const int ROLL_SERVO_PIN = 13;  // D7 is GPIO 13
const int PIDsampleTime = 10;   // PID sample time in milliseconds
MPU6050 mpu(Wire);

long timer = 0;

// For roll 100 seems to be centered, 199 max, 0 min
// For pitch, 85 seems to be centered, 199 max, 0 min

//  Pitch control variables:
//  p_sp - The desired pitch setpoint (e.g., the target angle)
//  p_in - The actual pitch input from a sensor (e.g., a gyroscope)
//  p_out - The calculated output value that will be sent to the actuator (servo)
double p_sp, p_in, p_out;
double r_sp, r_in, r_out;
// PID gain constants for pitch control:
// p_kp - Proportional gain constant for pitch control
// p_ki - Integral gain constant for pitch control
// p_kd - Derivative gain constant for pitch control
double p_kp = 0.05, p_ki = 0, p_kd = 0;
double r_kp = 0.05, r_ki = 0, r_kd = 0;
// Instantiate a PID controller object for pitch control
// The object takes the following parameters:
// - A pointer to the input variable (p_in)
// - A pointer to the output variable (p_out)
// - A pointer to the setpoint variable (p_sp)
// - The proportional gain constant (p_kp)
// - The integral gain constant (p_ki)
// - The derivative gain constant (p_kd)
// - The control direction (DIRECT)
PID pitch_pid(&p_in, &p_out, &p_sp, p_kp, p_ki, p_kd, DIRECT);
PID roll_pid(&r_in, &r_out, &r_sp, r_kp, r_ki, r_kd, DIRECT);

Servo p_servo;
Servo r_servo;

double p_servoGain = 1;// 1.38;
double r_servoGain = 1;//1.40;

double p_servoOffset = 85;
double r_servoOffset = 100;
void tbGyro()
{
  Serial.println("=======================================================");
  Serial.print("temp : ");
  Serial.println(mpu.getTemp());
  Serial.print("accX : ");
  Serial.print(mpu.getAccX());
  Serial.print("\taccY : ");
  Serial.print(mpu.getAccY());
  Serial.print("\taccZ : ");
  Serial.println(mpu.getAccZ());

  Serial.print("gyroX : ");
  Serial.print(mpu.getGyroX());
  Serial.print("\tgyroY : ");
  Serial.print(mpu.getGyroY());
  Serial.print("\tgyroZ : ");
  Serial.println(mpu.getGyroZ());

  Serial.print("accAngleX : ");
  Serial.print(mpu.getAccAngleX());
  Serial.print("\taccAngleY : ");
  Serial.println(mpu.getAccAngleY());

  Serial.print("gyroAngleX : ");
  Serial.print(mpu.getGyroAngleX());
  Serial.print("\tgyroAngleY : ");
  Serial.print(mpu.getGyroAngleY());
  Serial.print("\tgyroAngleZ : ");
  Serial.println(mpu.getGyroAngleZ());

  Serial.print("angleX : ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu.getAngleZ());
  Serial.println("=======================================================\n");
  timer = millis();
}

bool testConnectMPU()
{
  Adafruit_MPU6050 connectTestMPU;
  // Try to initialize!
  Serial.println("Trying to find the MPU6050");
  if (!connectTestMPU.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(500);
      ESP.reset();
    }
  }
  Serial.println("MPU6050 Found!");
  return true;
}
void setup()
{
  Serial.begin(9600);

  Wire.begin();

  // if (testConnectMPU())
  //{
  mpu.begin();
  //}
  mpu.setGyroOffsets(0.98, -2.80, 0.22);
  if (DEBUG)
  {
    Serial.println("Calibrating gyro...");
    mpu.calcGyroOffsets(true);
  }

  p_servo.attach(PITCH_SERVO_PIN);
  r_servo.attach(ROLL_SERVO_PIN);
  // delay(1000);
  // p_servo.write(160);
  // r_servo.write(160);
  // delay(1000);
  ////p_servo.write(0);
  ////r_servo.write(0);
  // delay(1000);

  // Set the initial setpoints for pitch and roll
  p_sp = 90; // Assuming the desired pitch angle is 0 degrees
  r_sp = 90; // Assuming the desired roll angle is 0 degrees

  // Set PID controllers' sample time and mode
  pitch_pid.SetSampleTime(PIDsampleTime);
  pitch_pid.SetMode(AUTOMATIC);
  roll_pid.SetSampleTime(PIDsampleTime);
  roll_pid.SetMode(AUTOMATIC);
  // setupArduinoOTA();
  Serial.println("Setup complete");
}

void updateReadings()
{

  // Read the mpu data
  p_in = mpu.getAngleX();
  r_in = mpu.getAngleY();

  // Compute the PID outputs
  pitch_pid.Compute();
  roll_pid.Compute();
}
void updateServos()
{
  r_servo.write(constrain((r_servoGain * (-1 * r_in) + r_servoOffset), 0, 199));
  p_servo.write(constrain((p_servoGain * (p_in) + p_servoOffset), 0, 199));
}

bool inputReadingCompleted = false;
String userInput = "";
void loop()
{
  mpu.update();     // Update the MPU sensor readings
  updateReadings(); // Update tDDhe PID inputs
  updateServos();   // Update the servos

  if (millis() - timer > 1000)
  {
    Serial.print("p_in : " + String(p_in) + ", p_out : " + String(p_out) + ", r_in : " + String(r_in) + ", r_out : " + String(r_out) + "\n");
    timer = millis();
  }
}