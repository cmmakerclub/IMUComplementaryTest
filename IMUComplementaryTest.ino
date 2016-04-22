#include "CurieIMU.h"

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

int calibrateOffsets = 1; // int to determine whether calibration takes place or not


//#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 16.348

#define M_PI 3.14159265359
#define dt (10.0/1000.0)             // 100hz = 10ms

#define RMotor_offset 60 // The offset of the Motor
#define LMotor_offset 50 // The offset of the Motor

// walkin
static float kp = 70.00f;
static float ki = 2.500f;
static float kd = 6000.00f;

uint32_t _prev = millis();

static uint32_t lastTime = millis();

int TN1 = 4;
int TN2 = 3;
int ENA = 5;
int TN3 = 8;
int TN4 = 7;
int ENB = 6;

static float error = 0;  // Proportion
static float errSum = 0;
static float dErr = 0;

struct Motor {
  float LOutput;
  float ROutput;
} motor_t;


//http://www.pieter-jan.com
static void ComplementaryFilter(int accData[3], int gyrData[3], float *pitch, float *roll)
{
  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
  *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;
  }
}

void calibrateIMU();
void readIMUSensor(float *Angle_Filtered);
void setup() {
  Serial.begin(115200); // initialize Serial communication


  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();
  calibrateIMU();

  // configure Arduino LED for activity indicator
  pinMode(ledPin, OUTPUT);
}

void loop() {
  static Motor motor;
  static float Angle_Filtered;
  if ( millis() - _prev > dt * 1000 ) {
    _prev = millis();
    readIMUSensor(&Angle_Filtered);
    //   If angle > 45 or < -45 then stop the robot
    if (abs(Angle_Filtered) < 45)
    {
      //      myPID(Angle_Filtered, &motor);
      //      PWMControl(motor.LOutput, motor.ROutput);
    }
    else
    {
      //TODO:// set zero when robot down.
      //      Output = error = errSum = dErr = 0;
      Serial.println("STOP");
      analogWrite(ENA, 0);  // Stop the wheels
      analogWrite(ENB, 0);  // Stop the wheels
    }
    blinkState = !blinkState;
    digitalWrite(ledPin, blinkState);
  }

}

void readIMUSensor(float *Angle_Filtered) {
  static float roll;
  static float pitch;
  // put your main code here, to run repeatedly:
  int accData[3];
  int gyrData[3];
  //  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
  CurieIMU.readMotionSensor(accData[0], accData[1], accData[2], gyrData[0], gyrData[1], gyrData[2]);
  ComplementaryFilter(accData, gyrData, &pitch, &roll);

  *Angle_Filtered = pitch;
  //  Serial.println(String("Angle Filter = ") + String(pitch));
  Serial.println(pitch);
}


void calibrateIMU() {
  //  calibrateGyro();
  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,128);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-4);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,127);
    //CurieIMU.setGyroOffset(X_AXIS,129);
    //CurieIMU.setGyroOffset(Y_AXIS,-1);
    //CurieIMU.setGyroOffset(Z_AXIS, 254);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }
}
