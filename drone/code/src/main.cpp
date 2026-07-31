#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

#define M1 0 // front-right CW
#define M2 1 // front-left CCW
#define M3 2 // rear-right CW
#define M4 3 // rear-left CCW
#define SDA_PIN 5
#define SCL_PIN 6
#define VBAT_PIN 4

// MPU-6050
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define GYRO_CFG 0x1B
#define ACCEL_CFG 0x1C
#define GYRO_SCALE 65.5f
#define ACCEL_SCALE 16384.0f

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
};

void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpu_init() {
  mpu_write(PWR_MGMT_1, 0x00);
  mpu_write(GYRO_CFG, 0x08);
  mpu_write(ACCEL_CFG, 0x00);
}

float gx_off=0, gy_off=0, gz_off=0;
float ax_off=0, ay_off=0, az_off=0;

void mpu_read(IMUData &d) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);

  int16_t ax_r = Wire.read()<<8 | Wire.read();
  int16_t ay_r = Wire.read()<<8 | Wire.read();
  int16_t az_r = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx_r = Wire.read()<<8 | Wire.read();
  int16_t gy_r = Wire.read()<<8 | Wire.read();
  int16_t gz_r = Wire.read()<<8 | Wire.read();

  d.ax = ax_r / ACCEL_SCALE;
  d.ay = ay_r / ACCEL_SCALE;
  d.az = az_r / ACCEL_SCALE;
  d.gx = gx_r / GYRO_SCALE;
  d.gy = gy_r / GYRO_SCALE;
  d.gz = gz_r / GYRO_SCALE;

  d.gx -= gx_off; d.gy -= gy_off; d.gz -= gz_off;
  d.ax -= ax_off; d.ay -= ay_off; d.az -= az_off;
}

void calibrate(int samples = 2000) {
  float sgx=0, sgy=0, sgz=0;
  float sax=0, say=0, saz=0;
  IMUData d;
  for (int i=0; i<samples; i++) {
    mpu_read(d);
    sgx += d.gx; sgy += d.gy; sgz += d.gz;
    sax += d.ax; say += d.ay; saz += d.az;
    delay(2);
  }
  gx_off = sgx/samples;
  gy_off = sgy/samples;
  gz_off = sgz/samples;
  ax_off = sax/samples;
  ay_off = say/samples;
  az_off = saz/samples - 1.0f;
}

struct PID {
  float kp, ki, kd;
  float integral;
  float prev_err;
  float i_max;
  
  float compute(float sp, float meas, float dt) {
    float err = sp - meas;
    integral = constrain(integral + err*dt, -i_max, i_max);
    float deriv = (err - prev_err) / dt;
    prev_err = err;
    return kp*err + ki*integral + kd*deriv;
  }
};

PID roll_pid = {1.5f,0.5f,0.05f,0,0,50};
PID pitch_pid = {1.5f,0.5f,0.05f,0,0,50};
PID yaw_pid = {2.0f,0.3f,0.00f,0,0,50};

#define PWM_FREQ 25000
#define PWM_RES 8

void motors_init() {
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcSetup(3, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1, 0);
  ledcAttachPin(M2, 1);
  ledcAttachPin(M3, 2);
  ledcAttachPin(M4, 3);
}

void set_motor(uint8_t ch, uint8_t duty) {
  ledcWrite(ch, duty);
}

void mix_and_write(float T, float R, float P, float Y) {
  float m1 = T + R - P - Y;
  float m2 = T - R - P + Y;
  float m3 = T + R + P + Y;
  float m4 = T - R + P - Y;
  
  auto clamp = [](float v) -> uint8_t {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
  };
  
  set_motor(0, clamp(m1));
  set_motor(1, clamp(m2));
  set_motor(2, clamp(m3));
  set_motor(3, clamp(m4));
}

#pragma pack(push, 1)
struct CtrlPacket {
  uint8_t throttle;
  int8_t roll;
  int8_t pitch;
  int8_t yaw;
  uint8_t armed;
  uint8_t mode;
  uint32_t seq;
};
#pragma pack(pop)

CtrlPacket rx_pkt = {};
volatile bool new_data = false;
unsigned long last_pkt = 0;

void on_recv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(CtrlPacket)) {
    memcpy(&rx_pkt, data, len);
    new_data = true;
    last_pkt = millis();
  }
}

void setup() {
  Serial.begin(115200);
  motors_init();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu_init();
  delay(200);
  calibrate(2000);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(on_recv);
}

void loop() {
  static unsigned long tp = micros();
  while (micros() - tp < 4000);
  float dt = (micros() - tp) * 1e-6f;
  tp = micros();

  IMUData imu;
  mpu_read(imu);

  if (millis() - last_pkt > 500 || !rx_pkt.armed) {
    mix_and_write(0, 0, 0, 0);
    return;
  }

  float sp_r = rx_pkt.roll * (150.0f / 127.0f);
  float sp_p = rx_pkt.pitch * (150.0f / 127.0f);
  float sp_y = rx_pkt.yaw * (150.0f / 127.0f);

  float r_out = roll_pid.compute(sp_r, imu.gx, dt);
  float p_out = pitch_pid.compute(sp_p, imu.gy, dt);
  float y_out = yaw_pid.compute(sp_y, imu.gz, dt);

  mix_and_write((float)rx_pkt.throttle, r_out, p_out, y_out);
}
