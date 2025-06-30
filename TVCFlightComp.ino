#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO08x bno08x(-1);  // No interrupt pin

Servo pitchServo;
Servo yawServo;

const int pitchPin = 6;
const int yawPin = 7;
sh2_SensorValue_t sensorValue;  // Global or local — up to you

// PID constants
float Kp = 0.5, Ki = 0.02, Kd = 0.6;

float pitch_integral = 0.0, yaw_integral = 0.0;
float pitch_prevErr = 0.0, yaw_prevErr = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  pitchServo.attach(pitchPin);
  yawServo.attach(yawPin);

  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not detected!");
    while (1);
  }

  bno08x.enableReport(SH2_ROTATION_VECTOR);
  lastTime = millis();
}

void loop() {
  sh2_SensorValue_t sensorValue;

  // Wait for new data
  if (!bno08x.getSensorEvent(&sensorValue)) return;

  // Only process rotation vector data
  if (sensorValue.sensorId != SH2_ROTATION_VECTOR) return;

  // Time delta
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Extract quaternion
  float qi = sensorValue.un.rotationVector.i;
  float qj = sensorValue.un.rotationVector.j;
  float qk = sensorValue.un.rotationVector.k;
  float qr = sensorValue.un.rotationVector.real;

  // Target orientation (identity quaternion)
  float ti = 0.0, tj = 0.0, tk = 0.0, tr = 1.0;

  // Quaternion error = target * inverse(current)
  float ei = tr * (-qi) + ti * qr + tj * qk - tk * qj;
  float ej = tr * (-qj) - ti * qk + tj * qr + tk * qi;
  float ek = tr * (-qk) + ti * qj - tj * qi + tk * qr; //Unused quaternion
  float er = tr * qr - ti * qi - tj * qj - tk * qk;

  // Axis-angle conversion
  float angle = 2.0 * acos(er);
  float sin_half_angle = sqrt(1.0 - er * er);
  float ux = 0, uy = 0;

  if (sin_half_angle > 0.001) {
    ux = ei / sin_half_angle; // pitch axis (X)
    uy = ej / sin_half_angle; // yaw axis (Y)
  }

  // Convert to degrees
  float pitchErr = ux * angle * 180.0 / PI;
  float yawErr   = uy * angle * 180.0 / PI;

  // --- PID: PITCH ---
  pitch_integral += pitchErr * dt;
  float pitch_deriv = (pitchErr - pitch_prevErr) / dt;
  float pitch_out = Kp * pitchErr + Ki * pitch_integral + Kd * pitch_deriv;
  pitch_prevErr = pitchErr;

  // --- PID: YAW ---
  yaw_integral += yawErr * dt;
  float yaw_deriv = (yawErr - yaw_prevErr) / dt;
  float yaw_out = Kp * yawErr + Ki * yaw_integral + Kd * yaw_deriv;
  yaw_prevErr = yawErr;

  // Convert PID output to servo angles
  float pitchAngle = constrain(90 + pitch_out, 0, 180);
  float yawAngle = constrain(90 + yaw_out, 0, 180);

  pitchServo.write(pitchAngle);
  yawServo.write(yawAngle);

  // Debug output
  Serial.print("PitchErr: "); Serial.print(pitchErr, 1);
  Serial.print(" YawErr: "); Serial.print(yawErr, 1);
  Serial.print(" | PitchOut: "); Serial.print(pitch_out, 1);
  Serial.print(" YawOut: "); Serial.println(yaw_out, 1);
}