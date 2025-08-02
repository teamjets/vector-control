#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

Servo servo_y1, servo_y2, servo_x1, servo_x2;
Adafruit_MPU6050 mpu;

const int neutral = 90;
const float pitchGain = 1.5;
const float yawGain = 1.5;
const float rollGain = 1.5;

float yawAngle = 0;
float rollAngle = 0;
unsigned long lastTime = 0;

void setup()
{
  servo_y1.attach(4);
  servo_y2.attach(16);
  servo_x1.attach(17);
  servo_x2.attach(5);

  Wire.begin();
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastTime = millis();
  delay(1000);
  Serial.begin(115200);
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float pitchAngle = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  yawAngle += g.gyro.y * dt * 180.0 / PI;
  rollAngle += g.gyro.z * dt * 180.0 / PI;

  int pitchCorrection = pitchAngle * pitchGain;
  int yawCorrection = yawAngle * yawGain;
  int rollCorrection = rollAngle * rollGain;

  servo_y1.write(constrain(neutral + pitchCorrection + rollCorrection, 0, 180));
  servo_y2.write(constrain(neutral - pitchCorrection + rollCorrection, 0, 180));
  servo_x1.write(constrain(neutral - yawCorrection + rollCorrection, 0, 180));
  servo_x2.write(constrain(neutral + yawCorrection + rollCorrection, 0, 180));

  Serial.println(yawAngle);
}

/* #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

Servo servo_y1, servo_y2, servo_x1, servo_x2;
Adafruit_MPU6050 mpu;

void setup(void)
{

  servo_y1.attach(4);
  servo_y2.attach(16);
  servo_x1.attach(17);
  servo_x2.attach(5);
  Wire.begin();
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  int y_dir = a.acceleration.y;
  int x_dir = a.acceleration.x;
  int z_dir = a.acceleration.z;

  y_dir = map(y_dir, -10, 10, 180, 0);
  servo_y1.write(y_dir);
  servo_y2.write(180 - y_dir);

  x_dir = map(x_dir, -10, 10, 180, 0);
  servo_x1.write(x_dir);
  servo_x2.write(180 - x_dir);


} */