#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Vibration motor control pin
int vibrationMotorPin = 13; //motor transistor is connected to pin 13

// Buzzer control pin
int buzzerPin = 12;

// Motor control pins
int in1 = 2;
int in2 = 3;
int enA = 9;  // Enable A, PWM pin

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(9600);
  pinMode (vibrationMotorPin, OUTPUT);
  pinMode (buzzerPin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if (a.acceleration.z > 4.6){
    digitalWrite(vibrationMotorPin, HIGH);
    // Move the motor forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 30);  // Adjust PWM value for desired speed
    }
  else{
    digitalWrite(vibrationMotorPin, LOW);
    // Stop the motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
 }
  if (a.acceleration.z > 6){
    tone(buzzerPin, 333 * a.acceleration.z); // Frequency increases as tilt increases
    delay(1200 / (a.acceleration.z * 2)); // Delay decreases as tilt increases
    noTone(buzzerPin); // Stop sound
    delay(1200 / (a.acceleration.z * 2)); // Delay decreases as tilt increases
  }
  else
    digitalWrite (buzzerPin, LOW); //stop buzzer
  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  /* Calculate roll and pitch */
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180/M_PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180/M_PI;

  /* Print out the values */
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.println(pitch);
  delay(500);
}
