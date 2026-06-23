/**
 * ═══════════════════════════════════════════════════════════════════════════
 * PLATAFORMA XDQ-001 — FLIGHT CONTROLLER FIRMWARE (V001)
 * ═══════════════════════════════════════════════════════════════════════════
 * Arquitectura: Híbrida Cooperativa Basada en Interrupciones (sin RTOS)
 * MCU: STM32F4 (con FPU habilitada)
 * Frecuencia Lazo Crítico: 1000Hz (Timer Interrupt, 1ms estricto)
 * Frecuencia Lazo Cinemático: 50Hz (Cooperative Background)
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ── DEFINICIÓN DE PINES (Asignación física de la placa XDQ-001) ──

// Bus SPI 3: Memoria Flash W25Q32JVSS
#define PIN_FLASH_SCK PB3
#define PIN_FLASH_MISO PB4
#define PIN_FLASH_MOSI PB5
#define PIN_FLASH_CS PA15

// Bus SPI 1: Sensores IMU, Magnetómetro, Barómetro
#define PIN_SPI1_SCK PA5
#define PIN_SPI1_MISO PA6
#define PIN_SPI1_MOSI PA7
#define PIN_ICM_CS PA4  // CS Giroscopio / Acelerómetro (ICM-42688)
#define PIN_MAG_CS PA3  // CS Magnetómetro (MMC5983MA)
#define PIN_BARO_CS PA0 // CS Barómetro (BMP280)
#define PIN_ICM_INT PA1 // Interrupción IMU
#define PIN_MAG_INT PA2 // Interrupción Magnetómetro

// Motores (ESC 4-in-1 - Salidas PWM)
#define PIN_MOTOR_1 PB6
#define PIN_MOTOR_2 PB7
#define PIN_MOTOR_3 PB8
#define PIN_MOTOR_4 PB9

// Bus SPI 2: Radio SX1280
#define PIN_RADIO_CS PB12
#define PIN_RADIO_SCK PB13
#define PIN_RADIO_MISO PB14
#define PIN_RADIO_MOSI PB15
#define PIN_RADIO_BUSY PA8
#define PIN_RADIO_RST PC13
#define PIN_RADIO_DIO1 PB0
#define PIN_RADIO_DIO2 PB1

// GPS NEO-6M (USART1)
#define PIN_GPS_RX PA10 // Conecta a TX del GPS (STM32 RX)
#define PIN_GPS_TX PA9  // Conecta a RX del GPS (STM32 TX)

// Debug LED
#define PIN_DEBUG_LED PC15

// ── INSTANCIAS DE BUSES SPI Y UART ──
SPIClass SPI_Sensors(PIN_SPI1_MOSI, PIN_SPI1_MISO, PIN_SPI1_SCK);
SPIClass SPI_Radio(PIN_RADIO_MOSI, PIN_RADIO_MISO, PIN_RADIO_SCK);
SPIClass SPI_Flash(PIN_FLASH_MOSI, PIN_FLASH_MISO, PIN_FLASH_SCK);

#define GPS_Serial Serial1

// ── HARDWARE TIMER (1000Hz Scheduler) ──
HardwareTimer *Timer_1kHz = nullptr;

// ── ESTRUCTURAS DE DATOS Y TELEMETRÍA ──

struct __attribute__((packed)) TelemetryData {
  int32_t latitude;  // grados × 1e7
  int32_t longitude; // grados × 1e7
  int16_t altitude;  // metros × 10
  int16_t heading;   // grados × 10 (0 a 3599)
  int16_t pitch;     // grados × 10
  int16_t roll;      // grados × 10
  int16_t gforce;    // G × 100
  int16_t velocityN; // m/s × 100
  int16_t velocityE; // m/s × 100
  int16_t velocityD; // m/s × 100
  uint8_t seq;       // Contador incremental
  uint8_t _pad[3];   // Padding
}; // Total: 28 bytes

struct __attribute__((packed)) SecureTelemetry {
  uint8_t magic; // 0xBB
  uint8_t seq;
  uint16_t crc;
  TelemetryData data;
}; // Total: 32 bytes (8 words)

struct __attribute__((packed)) SecureCommand {
  uint8_t magic; // 0xAA
  uint8_t seq;
  uint16_t crc;
  int16_t targetRoll;     // grados x 10
  int16_t targetPitch;    // grados x 10
  int16_t targetYaw;      // grados x 10
  int16_t targetThrottle; // 0 a 1000
  float homeLat;
  float homeLon;
  int16_t declinationX10;
  uint8_t navMode;  // 0=Manual, 1=Waypoint, 2=Orbit
  uint8_t cmdFlags; // Bit 0 = Reset route index
}; // Total: 24 bytes (6 words)

struct __attribute__((packed)) RouteWaypoint {
  float lat;
  float lon;
  float alt;
  uint8_t mode;      // 1=Waypoint, 2=Orbit
  uint8_t direction; // 0=CCW, 1=CW
  float radius;
}; // Total: 18 bytes

struct __attribute__((packed)) SecureWaypointPacket {
  uint8_t magic; // 0xAC
  uint8_t seq;
  uint16_t crc;
  uint8_t wpIndex; // 0..15
  uint8_t totalWps;
  uint8_t routeLoop; // 0=no, 1=yes
  uint8_t mode;      // NavigationMode (0=MANUAL, 1=WAYPOINT, 2=ORBIT)
  RouteWaypoint wp;
  uint8_t _pad[2]; // Pad a 28 bytes (7 words)
}; // Total: 28 bytes

// Estructura de registro Blackbox (Log de vuelo) - 32 bytes
struct __attribute__((packed)) LogPacket {
  uint32_t timestamp_us;
  int16_t roll_deg_x10;
  int16_t pitch_deg_x10;
  int16_t yaw_deg_x10;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  uint16_t motors[4];
  uint8_t flags;
  uint8_t padding;
};

// Clase Vector3D para Cálculos de Navegación Geométrica
class Vector3D {
public:
  float x, y, z;
  Vector3D(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}
  Vector3D operator-(const Vector3D &v) const {
    return Vector3D(x - v.x, y - v.y, z - v.z);
  }
  Vector3D operator+(const Vector3D &v) const {
    return Vector3D(x + v.x, y + v.y, z + v.z);
  }
  Vector3D operator*(float s) const { return Vector3D(x * s, y * s, z * s); }
  float magnitude() const { return sqrtf(x * x + y * y + z * z); }
  Vector3D normalize() const {
    float mag = magnitude();
    if (mag < 0.001f)
      return Vector3D(0, 0, 0);
    return Vector3D(x / mag, y / mag, z / mag);
  }
  float dot(const Vector3D &v) const { return x * v.x + y * v.y + z * v.z; }
  Vector3D cross(const Vector3D &v) const {
    return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }
};

// Configuración de Rutas de Navegación
const int MAX_WAYPOINTS = 16;
RouteWaypoint waypointRoute[MAX_WAYPOINTS];
int waypointCount = 0;
int waypointIndex = 0;
bool routeLoop = false;

enum NavigationMode { MODE_MANUAL = 0, MODE_WAYPOINT = 1, MODE_ORBIT = 2 };
NavigationMode currentMode = MODE_MANUAL;

const float WP_CAPTURE_NEAR = 8.0f; // metros
const float STALL_SPEED = 6.0f;     // m/s
float homeLat = 0.0f;
float homeLon = 0.0f;

// Estimador de Viento sin Pitot (Triángulo Estocástico en Virajes)
float windX = 0.0f; // Viento Este (m/s)
float windY = 0.0f; // Viento Norte (m/s)

// Buffers Ping-Pong Lock-free para Flash W25Q32JVSS
#define FLASH_PAGE_SIZE 256
volatile uint8_t flash_buffer_1[FLASH_PAGE_SIZE];
volatile uint8_t flash_buffer_2[FLASH_PAGE_SIZE];
volatile uint8_t *volatile active_write_buffer = flash_buffer_1;
volatile uint16_t buffer_index = 0;

volatile bool flash_buffer_ready_to_write = false;
volatile uint8_t *volatile dma_write_buffer = nullptr;
volatile uint32_t flash_write_address = 0;

// Variables Globales Compartidas
volatile float latitude = 0.0f;
volatile float longitude = 0.0f;
volatile float altitudeM = 0.0f;
volatile float velN = 0.0f, velE = 0.0f, velD = 0.0f;
volatile float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;
volatile float totalGForce = 1.0f;

volatile float setpoint_roll = 0.0f;
volatile float setpoint_pitch = 0.0f;
volatile float setpoint_yaw = 0.0f;
volatile float setpoint_throttle = 0.0f;

float magneticDeclinationDeg = -6.0f;
bool firstGPSFix = true;
float lastLat = 0.0f, lastLon = 0.0f, lastAlt = 0.0f;
uint32_t lastGPSVelTime = 0;

// Clave Compartida para Cifrado XXTEA y CRC
uint32_t sharedKey[4] = {0x58444630, 0x30314B45, 0x595F3230,
                         0x32362121}; // "XDF001KEY_2026!!"
#define XXTEA_DELTA 0x9e3779b9
#define XXTEA_MX                                                               \
  (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^                                   \
   ((sum ^ y) + (sharedKey[(p & 3) ^ e] ^ z)))

void btea(uint32_t *v, int n) {
  uint32_t y, z, sum;
  unsigned p, rounds, e;
  if (n > 1) {
    rounds = 6 + 52 / n;
    sum = 0;
    z = v[n - 1];
    do {
      sum += XXTEA_DELTA;
      e = (sum >> 2) & 3;
      for (p = 0; p < n - 1; p++) {
        y = v[p + 1];
        z = v[p] += XXTEA_MX;
      }
      y = v[0];
      z = v[n - 1] += XXTEA_MX;
    } while (--rounds);
  } else if (n < -1) {
    n = -n;
    rounds = 6 + 52 / n;
    sum = rounds * XXTEA_DELTA;
    y = v[0];
    do {
      e = (sum >> 2) & 3;
      for (p = n - 1; p > 0; p--) {
        z = v[p - 1];
        y = v[p] -= XXTEA_MX;
      }
      z = v[n - 1];
      y = v[0] -= XXTEA_MX;
      sum -= XXTEA_DELTA;
    } while (--rounds);
  }
}

uint16_t calculateCRC16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

// ── COMPONENTES DE CONTROL PID DE ANCHO DE BANDA AISLADO ──

class LPF_PT1 {
private:
  float val = 0.0f;
  float alpha = 1.0f;

public:
  void set_cutoff(float cutoff_hz, float sample_rate_hz) {
    float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    float dt = 1.0f / sample_rate_hz;
    alpha = dt / (rc + dt);
  }
  float update(float input) {
    val = val + alpha * (input - val);
    return val;
  }
};

class FlightPID {
private:
  float kp, ki, kd, kff;
  float integral = 0.0f;
  float limit_i;
  LPF_PT1 d_filter;
  LPF_PT1 p_filter;

public:
  FlightPID(float p, float i, float d, float ff, float lim_i, float cutoff_p,
            float cutoff_d)
      : kp(p), ki(i), kd(d), kff(ff), limit_i(lim_i) {
    p_filter.set_cutoff(cutoff_p, 1000.0f);
    d_filter.set_cutoff(cutoff_d, 1000.0f);
  }
  float compute(float setpoint, float measured, float rate, float dt) {
    float error = setpoint - measured;
    float p_out = p_filter.update(kp * error);
    integral = constrain(integral + (ki * error * dt), -limit_i, limit_i);
    float d_out = d_filter.update(-kd * rate);
    float ff_out = kff * setpoint;
    return p_out + integral + d_out + ff_out;
  }
};

FlightPID pid_roll(1.3f, 0.05f, 0.15f, 0.3f, 15.0f, 150.0f, 25.0f);
FlightPID pid_pitch(1.3f, 0.05f, 0.15f, 0.3f, 15.0f, 150.0f, 25.0f);
FlightPID pid_yaw(2.0f, 0.02f, 0.0f, 0.1f, 10.0f, 150.0f, 10.0f);

// ── SISTEMA DE NAVEGACIÓN DUAL-LAYER: MAHONY AHRS (1000Hz) ──

class MahonyAHRS {
public:
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float twoKp = 4.0f;
  float twoKi = 0.01f;
  float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float dt) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.001f)
      return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - 0.5f + q3 * q3;

    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    if (twoKi > 0.0f) {
      integralFBx += twoKi * ex * dt;
      integralFBy += twoKi * ey * dt;
      integralFBz += twoKi * ez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    float recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }

  float getRoll() const {
    return atan2f(2.0f * (q0 * q1 + q2 * q3),
                  q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
           57.2957795f;
  }
  float getPitch() const {
    return asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.2957795f;
  }
  float getYaw() const {
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
           57.2957795f;
  }
};

MahonyAHRS attitude_ahrs;

// ── SISTEMA DE NAVEGACIÓN DUAL-LAYER: ESKF (50Hz) ──

class KinematicsKF3D {
public:
  float x[6]; // [posN, velN, posE, velE, posD, velD]
  float P[6];
  const float Qa = 0.4f;
  const float Rp = 3.5f;
  const float Rv = 0.2f;

  KinematicsKF3D() {
    for (int i = 0; i < 6; i++) {
      x[i] = 0.0f;
      P[i] = 50.0f;
    }
  }
  void predict(float aN, float aE, float aD, float dt) {
    float dt2 = dt * dt;
    x[0] += x[1] * dt + 0.5f * aN * dt2;
    x[1] += aN * dt;
    P[0] += P[1] * dt * 2.0f + Qa * dt2 * dt2 * 0.25f;
    P[1] += Qa * dt2;
    x[2] += x[3] * dt + 0.5f * aE * dt2;
    x[3] += aE * dt;
    P[2] += P[3] * dt * 2.0f + Qa * dt2 * dt2 * 0.25f;
    P[3] += Qa * dt2;
    x[4] += x[5] * dt + 0.5f * aD * dt2;
    x[5] += aD * dt;
    P[4] += P[5] * dt * 2.0f + Qa * dt2 * dt2 * 0.25f;
    P[5] += Qa * dt2;
  }
  void updateGPS(float posN, float posE, float posD, float vN, float vE,
                 float vD) {
    _updateScalar(0, posN, Rp);
    _updateScalar(1, vN, Rv);
    _updateScalar(2, posE, Rp);
    _updateScalar(3, vE, Rv);
    _updateScalar(4, posD, Rp);
    _updateScalar(5, vD, Rv);
  }

private:
  void _updateScalar(int idx, float z, float R) {
    float K = P[idx] / (P[idx] + R);
    x[idx] += K * (z - x[idx]);
    P[idx] *= (1.0f - K);
  }
};

KinematicsKF3D kf3d;

// ── NAVEGACIÓN POR WAYPOINTS L1 ADAPTATIVA Y ORBITAL (CADI_A) ──

Vector3D GPSToLocal(float lat, float lon, float alt) {
  if (homeLat == 0.0f || homeLon == 0.0f)
    return Vector3D(0, 0, 0);

  float dLat = (lat - homeLat) * 111320.0f;
  float dLon = (lon - homeLon) * 111320.0f * cosf(lat * 3.14159265f / 180.0f);
  float dAlt = alt; // Altitud relativa
  return Vector3D(dLon, dLat, dAlt);
}

void estimateWind() {
  float rLocal, pLocal, yLocal;
  noInterrupts();
  rLocal = rollDeg;
  pLocal = pitchDeg;
  yLocal = yawDeg;
  interrupts();

  if (fabsf(rLocal) < 10.0f)
    return;

  float groundSpeed = sqrtf(velE * velE + velN * velN);
  if (groundSpeed < 3.0f)
    return;

  float pitchRad = pLocal * 0.0174533f;
  float yawRad = yLocal * 0.0174533f;

  float noseDirX = cosf(pitchRad) * sinf(yawRad); // Este
  float noseDirY = cosf(pitchRad) * cosf(yawRad); // Norte

  float rawWindX = velE - groundSpeed * noseDirX;
  float rawWindY = velN - groundSpeed * noseDirY;

  windX = windX * 0.95f + rawWindX * 0.05f;
  windY = windY * 0.95f + rawWindY * 0.05f;
}

void runL1Navigation() {
  if (waypointCount == 0) {
    noInterrupts();
    setpoint_roll = 0;
    setpoint_pitch = 3.0f;
    setpoint_throttle = 1500; // PWM escalado
    noInterrupts();
    return;
  }

  Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
  Vector3D velocity(velE, velN, -velD);
  float speed = max(velocity.magnitude(), 2.0f);

  RouteWaypoint currentWP = waypointRoute[waypointIndex];
  Vector3D B = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);

  float distToWP = (B - pos).magnitude();
  if (distToWP < WP_CAPTURE_NEAR) {
    waypointIndex++;
    if (waypointIndex >= waypointCount) {
      if (routeLoop) {
        waypointIndex = 0;
      } else {
        waypointIndex = waypointCount - 1;
        currentWP.mode = 2; // Órbita Loiter en el último WP
        currentWP.radius = 30.0f;
        currentWP.direction = 1;
      }
    }
    currentWP = waypointRoute[waypointIndex];
    B = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);
  }

  Vector3D A;
  if (waypointIndex == 0) {
    A = Vector3D(0, 0, currentWP.alt);
  } else {
    RouteWaypoint prevWP = waypointRoute[waypointIndex - 1];
    A = GPSToLocal(prevWP.lat, prevWP.lon, prevWP.alt);
  }

  Vector3D AB = B - A;
  float segLen = AB.magnitude();
  if (segLen < 1.0f) {
    A = pos;
    AB = B - A;
    segLen = AB.magnitude();
  }

  Vector3D AB_norm = AB.normalize();
  Vector3D AP = pos - A;
  float crossTrackErr = AP.x * AB_norm.y - AP.y * AB_norm.x;

  float L1_dist = 3.0f * speed;
  L1_dist = constrain(L1_dist, 10.0f, 60.0f);

  float projDist = AP.dot(AB_norm);
  float targetDistOnSeg =
      projDist +
      sqrtf(max(0.0f, L1_dist * L1_dist - crossTrackErr * crossTrackErr));
  targetDistOnSeg = constrain(targetDistOnSeg, 0.0f, segLen);

  Vector3D targetL1 = A + AB_norm * targetDistOnSeg;
  Vector3D toTargetL1 = targetL1 - pos;

  float yLocal;
  noInterrupts();
  yLocal = yawDeg;
  interrupts();

  float yawRad = yLocal * 0.0174533f;
  Vector3D headingDir(sinf(yawRad), cosf(yawRad), 0);

  float dot = headingDir.dot(toTargetL1.normalize());
  dot = constrain(dot, -1.0f, 1.0f);
  float eta = acosf(dot);
  if (toTargetL1.x * headingDir.y - toTargetL1.y * headingDir.x < 0) {
    eta = -eta;
  }

  float a_lat = 2.0f * speed * speed / L1_dist * sinf(eta);
  float rollL1 = atanf(a_lat / 9.81f) * 57.2957795f;
  float rollOut = constrain(rollL1, -35.0f, 35.0f);

  // Compensación de viento cruzado
  float crosswind = -windX * cosf(yawRad) + windY * sinf(yawRad);
  rollOut += constrain(crosswind * 1.5f, -10.0f, 10.0f);
  rollOut = constrain(rollOut, -35.0f, 35.0f);

  // TECS simplificado
  float altErr = currentWP.alt - altitudeM;
  float speedErr = 12.0f - speed;
  float energyTotalErr = altErr * 0.1f + speedErr * 0.2f;
  float energyDistErr = altErr * 0.1f - speedErr * 0.2f;

  float throttleTECS =
      2048.0f + energyTotalErr * 800.0f; // Mitad del rango PWM (0-4095)
  float pitchTECS = energyDistErr * 15.0f;

  noInterrupts();
  setpoint_roll = rollOut;
  setpoint_pitch = constrain(pitchTECS, -10.0f, 10.0f);
  setpoint_throttle = constrain((int)throttleTECS, 1000, 3200);
  setpoint_yaw = 0.0f;
  interrupts();
}

void runOrbitNavigation() {
  Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
  RouteWaypoint currentWP = waypointRoute[waypointIndex];
  Vector3D center = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);
  float radius = max(currentWP.radius, 15.0f);
  bool cw = (currentWP.direction == 1);

  Vector3D toCenter = center - pos;
  float distToCenter = toCenter.magnitude();
  float radiusErr = distToCenter - radius;

  static float lastRadiusErr = 0;
  float radiusDeriv = radiusErr - lastRadiusErr;
  lastRadiusErr = radiusErr;

  float baseRoll = cw ? -20.0f : 20.0f;
  float rollCorr =
      (radiusErr * 0.6f + radiusDeriv * 0.2f) * (cw ? 1.0f : -1.0f);
  float rollOrbit = baseRoll + rollCorr;

  float altErr = currentWP.alt - altitudeM;
  float pitchOrbit = constrain(altErr * 0.5f, -10.0f, 10.0f);

  float rLocal;
  noInterrupts();
  rLocal = rollDeg;
  interrupts();

  float cosRoll = cosf(rLocal * 0.0174533f);
  if (cosRoll < 0.2f)
    cosRoll = 0.2f;
  float throttleCoordinated = 2048.0f / cosRoll;

  noInterrupts();
  setpoint_roll = constrain(rollOrbit, -35.0f, 35.0f);
  setpoint_pitch = pitchOrbit;
  setpoint_throttle = constrain((int)throttleCoordinated, 1200, 3000);
  setpoint_yaw = 0.0f;
  interrupts();
}

void applySafetyGuards() {
  float rLocal, gLocal, srLocal, spLocal;
  noInterrupts();
  rLocal = rollDeg;
  gLocal = totalGForce;
  srLocal = setpoint_roll;
  spLocal = setpoint_pitch;
  interrupts();

  // 1. Dynamic Stall Guard
  float cosRoll = cosf(rLocal * 0.0174533f);
  if (cosRoll < 0.2f)
    cosRoll = 0.2f;
  float v_stall_din = STALL_SPEED / sqrtf(cosRoll);

  float speed = sqrtf(velE * velE + velN * velN + velD * velD);
  if (speed < v_stall_din) {
    noInterrupts();
    setpoint_roll = 0.0f;
    setpoint_pitch = -5.0f; // Nariz abajo para recuperar sustentación
    setpoint_throttle = 3500;
    setpoint_yaw = 0.0f;
    interrupts();
    return;
  }

  // 2. G-Limiter
  const float G_SOFT_START = 2.0f;
  const float G_SOFT_END = 3.5f;
  if (gLocal > G_SOFT_START) {
    float attenuation =
        1.0f - constrain((gLocal - G_SOFT_START) / (G_SOFT_END - G_SOFT_START),
                         0.0f, 0.7f);
    noInterrupts();
    setpoint_roll = srLocal * attenuation;
    setpoint_pitch = spLocal * attenuation;
    interrupts();
  }
}

// ── DRIVER DE RADIO SX1280 (FHSS Y DUAL-PIPE) ──

const uint8_t fhssChannels[8] = {12, 28, 44, 60, 76, 92, 108, 124};
uint8_t currentChannelIdx = 0;
uint32_t lastPacketReceivedMs = 0;
SecureTelemetry secTelem;
SecureCommand secCmd;
SecureWaypointPacket secWpPkt;
uint8_t telemSeq = 0;

void sx1280_set_channel(uint8_t ch) {
  SPI_Radio.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_RADIO_CS, LOW);
  SPI_Radio.transfer(0x18);
  SPI_Radio.transfer(ch);
  digitalWrite(PIN_RADIO_CS, HIGH);
  SPI_Radio.endTransaction();
}

void send_telemetry_packet() {
  TelemetryData telem;
  telem.latitude = (int32_t)(latitude * 1e7f);
  telem.longitude = (int32_t)(longitude * 1e7f);
  telem.altitude = (int16_t)(altitudeM * 10.0f);
  noInterrupts();
  telem.heading = (int16_t)(yawDeg * 10.0f);
  telem.pitch = (int16_t)(pitchDeg * 10.0f);
  telem.roll = (int16_t)(rollDeg * 10.0f);
  telem.gforce = (int16_t)(totalGForce * 100.0f);
  noInterrupts();
  telem.velocityN = (int16_t)(velN * 100.0f);
  telem.velocityE = (int16_t)(velE * 100.0f);
  telem.velocityD = (int16_t)(velD * 100.0f);
  telem.seq = telemSeq++;

  secTelem.magic = 0xBB;
  secTelem.seq = telem.seq;
  secTelem.data = telem;
  secTelem.crc =
      calculateCRC16((uint8_t *)&secTelem.data, sizeof(TelemetryData));

  btea((uint32_t *)&secTelem, 8);

  SPI_Radio.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_RADIO_CS, LOW);
  SPI_Radio.transfer(0x0A);
  uint8_t *telemPtr = (uint8_t *)&secTelem;
  for (size_t i = 0; i < sizeof(SecureTelemetry); i++) {
    SPI_Radio.transfer(telemPtr[i]);
  }
  digitalWrite(PIN_RADIO_CS, HIGH);
  SPI_Radio.endTransaction();
}

void handle_radio_comms() {
  // FHSS Hopping
  if (millis() - lastPacketReceivedMs > 1000) {
    static uint32_t lastHopMs = 0;
    if (millis() - lastHopMs > 100) {
      currentChannelIdx = (currentChannelIdx + 1) % 8;
      sx1280_set_channel(fhssChannels[currentChannelIdx]);
      lastHopMs = millis();
    }
  }

  // Recepción DIO1 (Comandos o Waypoints)
  if (digitalRead(PIN_RADIO_DIO1) == HIGH) {
    // Leer el búfer RX
    uint8_t rawBuffer[32];
    SPI_Radio.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_RADIO_CS, LOW);
    SPI_Radio.transfer(0x0B); // Leer comando RX Buffer
    for (size_t i = 0; i < 32; i++) {
      rawBuffer[i] = SPI_Radio.transfer(0x00);
    }
    digitalWrite(PIN_RADIO_CS, HIGH);
    SPI_Radio.endTransaction();

    uint8_t magic = rawBuffer[0];

    // Diferenciar tuberías por Magic Header
    if (magic == 0xAA) {
      // Pipe 2: Comando
      memcpy(&secCmd, rawBuffer, sizeof(SecureCommand));
      btea((uint32_t *)&secCmd, -6);
      uint16_t crc = calculateCRC16((uint8_t *)&secCmd.targetRoll, 20);

      if (secCmd.magic == 0xAA && secCmd.crc == crc) {
        lastPacketReceivedMs = millis();
        currentMode = (NavigationMode)secCmd.navMode;
        if (secCmd.cmdFlags & 0x01) {
          waypointIndex = 0;
        }

        if (currentMode == MODE_MANUAL) {
          noInterrupts();
          setpoint_roll = secCmd.targetRoll / 10.0f;
          setpoint_pitch = secCmd.targetPitch / 10.0f;
          setpoint_yaw = secCmd.targetYaw / 10.0f;
          setpoint_throttle = secCmd.targetThrottle * 4.095f;
          interrupts();
        }
        magneticDeclinationDeg = secCmd.declinationX10 / 10.0f;
        if (secCmd.homeLat != 0.0f && secCmd.homeLon != 0.0f) {
          homeLat = secCmd.homeLat;
          homeLon = secCmd.homeLon;
        }
      }
    } else if (magic == 0xAC) {
      // Pipe 2: Waypoint
      memcpy(&secWpPkt, rawBuffer, sizeof(SecureWaypointPacket));
      btea((uint32_t *)&secWpPkt, -7);
      uint16_t crc = calculateCRC16((uint8_t *)&secWpPkt.wpIndex, 24);

      if (secWpPkt.magic == 0xAC && secWpPkt.crc == crc) {
        lastPacketReceivedMs = millis();
        uint8_t idx = secWpPkt.wpIndex;
        if (idx < MAX_WAYPOINTS) {
          waypointRoute[idx] = secWpPkt.wp;
          waypointCount = secWpPkt.totalWps;
          routeLoop = (secWpPkt.routeLoop == 1);
          currentMode = (NavigationMode)secWpPkt.mode;
          if (idx == 0) {
            waypointIndex = 0;
          }
        }
      }
    }
  }
}

// ── DETECCIÓN FÍSICA DE SENSORES Y LECTURAS ICM-42688 ──

void icm42688_init() {
  SPI_Sensors.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ICM_CS, LOW);
  SPI_Sensors.transfer(0x11);
  SPI_Sensors.transfer(0x28);
  digitalWrite(PIN_ICM_CS, HIGH);
  delay(10);
  digitalWrite(PIN_ICM_CS, LOW);
  SPI_Sensors.transfer(0x12);
  SPI_Sensors.transfer(0x28);
  digitalWrite(PIN_ICM_CS, HIGH);
  SPI_Sensors.endTransaction();
}

void icm42688_read_raw(float &ax, float &ay, float &az, float &gx, float &gy,
                       float &gz) {
  SPI_Sensors.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ICM_CS, LOW);
  SPI_Sensors.transfer(0x1F | 0x80);
  int16_t raw_ax =
      (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  int16_t raw_ay =
      (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  int16_t raw_az =
      (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  int16_t raw_gx =
      (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  int16_t raw_gy =
      (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  int16_t raw_gz =
      (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  digitalWrite(PIN_ICM_CS, HIGH);
  SPI_Sensors.endTransaction();

  ax = (float)raw_ax / 4096.0f;
  ay = (float)raw_ay / 4096.0f;
  az = (float)raw_az / 4096.0f;
  gx = (float)raw_gx * 0.0010642f;
  gy = (float)raw_gy * 0.0010642f;
  gz = (float)raw_gz * 0.0010642f;
}

// ── CONTROL DE MOTORES (ESC PWM 400Hz) ──

void init_motors() {
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);
  pinMode(PIN_MOTOR_3, OUTPUT);
  pinMode(PIN_MOTOR_4, OUTPUT);
  analogWriteFrequency(400);
  analogWriteResolution(12);
}

void write_motors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  analogWrite(PIN_MOTOR_1, m1);
  analogWrite(PIN_MOTOR_2, m2);
  analogWrite(PIN_MOTOR_3, m3);
  analogWrite(PIN_MOTOR_4, m4);
}

// ── RUTINA DE SERVICIO DE INTERRUPCIÓN (1000Hz TIMER ISR) ──

void Timer_ISR() {
  uint32_t t_now = micros();

  float ax, ay, az, gx, gy, gz;
  icm42688_read_raw(ax, ay, az, gx, gy, gz);

  // 1. Navegación (Mahony AHRS 1000Hz)
  attitude_ahrs.update(gx, gy, gz, ax, ay, az, 0.001f);
  float roll = attitude_ahrs.getRoll();
  float pitch = attitude_ahrs.getPitch();
  float yaw = attitude_ahrs.getYaw();

  rollDeg = roll;
  pitchDeg = pitch;
  yawDeg = yaw;

  // 2. Control de Vuelo (PID)
  float out_roll = pid_roll.compute(setpoint_roll, roll, gx, 0.001f);
  float out_pitch = pid_pitch.compute(setpoint_pitch, pitch, gy, 0.001f);
  float out_yaw = pid_yaw.compute(setpoint_yaw, yaw, gz, 0.001f);

  // 3. Mezclador Quadcopter X + G-Limits
  float throttle = setpoint_throttle;
  float m1_val = throttle - out_roll + out_pitch - out_yaw; // Front Right
  float m2_val = throttle - out_roll - out_pitch + out_yaw; // Rear Right
  float m3_val = throttle + out_roll - out_pitch - out_yaw; // Rear Left
  float m4_val = throttle + out_roll + out_pitch + out_yaw; // Front Left

  float max_motor = max(max(m1_val, m2_val), max(m3_val, m4_val));
  if (max_motor > 4095.0f) {
    float shift = max_motor - 4095.0f;
    m1_val -= shift;
    m2_val -= shift;
    m3_val -= shift;
    m4_val -= shift;
  }

  uint16_t m1 = constrain((int)m1_val, 0, 4095);
  uint16_t m2 = constrain((int)m2_val, 0, 4095);
  uint16_t m3 = constrain((int)m3_val, 0, 4095);
  uint16_t m4 = constrain((int)m4_val, 0, 4095);

  write_motors(m1, m2, m3, m4);

  // 4. Guardar datos en Buffer Ping-Pong
  if (!flash_buffer_ready_to_write) {
    LogPacket packet;
    packet.timestamp_us = t_now;
    packet.roll_deg_x10 = (int16_t)(roll * 10.0f);
    packet.pitch_deg_x10 = (int16_t)(pitch * 10.0f);
    packet.yaw_deg_x10 = (int16_t)(yaw * 10.0f);
    packet.gyro_x = (int16_t)(gx * 1000.0f);
    packet.gyro_y = (int16_t)(gy * 1000.0f);
    packet.gyro_z = (int16_t)(gz * 1000.0f);
    packet.accel_x = (int16_t)(ax * 1000.0f);
    packet.accel_y = (int16_t)(ay * 1000.0f);
    packet.accel_z = (int16_t)(az * 1000.0f);
    packet.motors[0] = m1;
    packet.motors[1] = m2;
    packet.motors[2] = m3;
    packet.motors[3] = m4;
    packet.flags = 0;
    packet.padding = 0;

    uint8_t *ptr = (uint8_t *)&packet;
    for (uint8_t i = 0; i < sizeof(LogPacket); i++) {
      active_write_buffer[buffer_index++] = ptr[i];
    }

    if (buffer_index >= FLASH_PAGE_SIZE) {
      dma_write_buffer = active_write_buffer;
      active_write_buffer = (active_write_buffer == flash_buffer_1)
                                ? flash_buffer_2
                                : flash_buffer_1;
      buffer_index = 0;
      flash_buffer_ready_to_write = true;
    }
  }
}

// ── LECTURAS ADICIONALES Y PARSEADOR GPS DE SEGUNDO PLANO ──

void read_magnetometer() {
  SPI_Sensors.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_MAG_CS, LOW);
  SPI_Sensors.transfer(0x06);
  int16_t mx = (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  int16_t my = (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  int16_t mz = (SPI_Sensors.transfer(0x00) << 8) | SPI_Sensors.transfer(0x00);
  digitalWrite(PIN_MAG_CS, HIGH);
  SPI_Sensors.endTransaction();
}

void read_barometer() {
  SPI_Sensors.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_BARO_CS, LOW);
  SPI_Sensors.transfer(0xF7 | 0x80);
  int32_t raw_p = ((uint32_t)SPI_Sensors.transfer(0x00) << 12) |
                  ((uint32_t)SPI_Sensors.transfer(0x00) << 4) |
                  (SPI_Sensors.transfer(0x00) >> 4);
  digitalWrite(PIN_BARO_CS, HIGH);
  SPI_Sensors.endTransaction();

  altitudeM = 44330.0f * (1.0f - powf((float)raw_p / 101325.0f, 0.1903f));
}

void process_gps() {
  static char nmeaBuf[120];
  static uint8_t nmeaIdx = 0;

  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    if (c == '$') {
      nmeaIdx = 0;
      nmeaBuf[nmeaIdx++] = c;
    } else if (c == '\n' || c == '\r') {
      nmeaBuf[nmeaIdx] = '\0';
      if (nmeaIdx > 10 && strncmp(nmeaBuf, "$GPRMC", 6) == 0) {
        float latVal = 0.0f, lonVal = 0.0f, speedKts = 0.0f;
        char latInd = 'N', lonInd = 'W';
        int fieldIdx = 0;
        char *token = strtok(nmeaBuf, ",");
        while (token != NULL) {
          fieldIdx++;
          if (fieldIdx == 4)
            latVal = atof(token);
          else if (fieldIdx == 5)
            latInd = token[0];
          else if (fieldIdx == 6)
            lonVal = atof(token);
          else if (fieldIdx == 7)
            lonInd = token[0];
          else if (fieldIdx == 8)
            speedKts = atof(token);
          token = strtok(NULL, ",");
        }

        float latDeg = (int)(latVal / 100);
        float latMin = latVal - (latDeg * 100);
        float lat = latDeg + (latMin / 60.0f);
        if (latInd == 'S')
          lat = -lat;

        float lonDeg = (int)(lonVal / 100);
        float lonMin = lonVal - (lonDeg * 100);
        float lon = lonDeg + (lonMin / 60.0f);
        if (lonInd == 'W')
          lon = -lon;

        latitude = lat;
        longitude = lon;

        uint32_t now = millis();
        float dt = (now - lastGPSVelTime) / 1000.0f;
        if (!firstGPSFix && dt > 0.05f) {
          float dNorth = (lat - lastLat) * 111320.0f;
          float dEast = (lon - lastLon) * 111320.0f * cosf(lat * 0.0174533f);
          velE = dEast / dt;
          velN = dNorth / dt;

          kf3d.updateGPS(kf3d.x[0] + dNorth, kf3d.x[2] + dEast, kf3d.x[4], velN,
                         velE, velD);
        }
        lastLat = lat;
        lastLon = lon;
        lastGPSVelTime = now;
        firstGPSFix = false;
      }
      nmeaIdx = 0;
    } else if (nmeaIdx < sizeof(nmeaBuf) - 1) {
      nmeaBuf[nmeaIdx++] = c;
    }
  }
}

// ── GESTIÓN DE FLASH W25Q32JVSS NO BLOQUEANTE ──

bool w25q32_is_busy() {
  SPI_Flash.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI_Flash.transfer(0x05);
  uint8_t status = SPI_Flash.transfer(0x00);
  digitalWrite(PIN_FLASH_CS, HIGH);
  SPI_Flash.endTransaction();
  return (status & 0x01);
}

void w25q32_write_page(uint32_t address, const uint8_t *data) {
  SPI_Flash.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI_Flash.transfer(0x06);
  digitalWrite(PIN_FLASH_CS, HIGH);
  SPI_Flash.endTransaction();

  SPI_Flash.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI_Flash.transfer(0x02);
  SPI_Flash.transfer((address >> 16) & 0xFF);
  SPI_Flash.transfer((address >> 8) & 0xFF);
  SPI_Flash.transfer(address & 0xFF);
  for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) {
    SPI_Flash.transfer(data[i]);
  }
  digitalWrite(PIN_FLASH_CS, HIGH);
  SPI_Flash.endTransaction();
}

void handle_flash_logging() {
  static enum { IDLE, WAITING_FOR_READY } state = IDLE;
  switch (state) {
  case IDLE:
    if (flash_buffer_ready_to_write) {
      if (!w25q32_is_busy()) {
        w25q32_write_page(flash_write_address,
                          (const uint8_t *)dma_write_buffer);
        state = WAITING_FOR_READY;
      }
    }
    break;
  case WAITING_FOR_READY:
    if (!w25q32_is_busy()) {
      flash_write_address += FLASH_PAGE_SIZE;
      flash_buffer_ready_to_write = false;
      state = IDLE;
    }
    break;
  }
}

// ── SETUP & INITIALIZATION ──

void setup() {
  pinMode(PIN_DEBUG_LED, OUTPUT);
  digitalWrite(PIN_DEBUG_LED, HIGH);

  Serial.begin(115200);
  GPS_Serial.begin(9600);

  pinMode(PIN_ICM_CS, OUTPUT);
  pinMode(PIN_MAG_CS, OUTPUT);
  pinMode(PIN_BARO_CS, OUTPUT);
  pinMode(PIN_RADIO_CS, OUTPUT);
  pinMode(PIN_FLASH_CS, OUTPUT);
  digitalWrite(PIN_ICM_CS, HIGH);
  digitalWrite(PIN_MAG_CS, HIGH);
  digitalWrite(PIN_BARO_CS, HIGH);
  digitalWrite(PIN_RADIO_CS, HIGH);
  digitalWrite(PIN_FLASH_CS, HIGH);

  SPI_Sensors.begin();
  SPI_Radio.begin();
  SPI_Flash.begin();

  icm42688_init();
  init_motors();

#if defined(ARDUINO_ARCH_STM32)
  Timer_1kHz = new HardwareTimer(TIM2);
  Timer_1kHz->setOverflow(1000, HERTZ_FORMAT);
  Timer_1kHz->attachInterrupt(Timer_ISR);
  Timer_1kHz->resume();
#endif

  digitalWrite(PIN_DEBUG_LED, LOW);
}

// ── LOOP PRINCIPAL COOPERATIVO (FONDO - 50Hz) ──

void loop() {
  static uint32_t last_loop_50hz = 0;
  uint32_t now = millis();

  // Tareas de Lazo de Fondo a 50Hz
  if (now - last_loop_50hz >= 20) {
    float dt = (now - last_loop_50hz) / 1000.0f;
    last_loop_50hz = now;

    digitalWrite(PIN_DEBUG_LED, !digitalRead(PIN_DEBUG_LED));

    // 1. Navegación Lazo Externo (Lecturas y ESKF Inercial a 50Hz)
    read_magnetometer();
    read_barometer();
    process_gps();

    // Rotación de aceleraciones a Earth-Frame y predicción de Kalman
    float q0 = attitude_ahrs.q0, q1 = attitude_ahrs.q1, q2 = attitude_ahrs.q2,
          q3 = attitude_ahrs.q3;
    float ax = (1 - 2 * (q2 * q2 + q3 * q3)) * ax_g +
               2 * (q1 * q2 - q0 * q3) * ay_g + 2 * (q1 * q3 + q0 * q2) * az_g;
    float ay = 2 * (q1 * q2 + q0 * q3) * ax_g +
               (1 - 2 * (q1 * q1 + q3 * q3)) * ay_g +
               2 * (q2 * q3 - q0 * q1) * az_g;
    float az = 2 * (q1 * q3 - q0 * q2) * ax_g + 2 * (q2 * q3 + q0 * q1) * ay_g +
               (1 - 2 * (q1 * q1 + q2 * q2)) * az_g;

    float aN = ay;
    float aE = ax;
    float aD = -(az - 9.81f);
    kf3d.predict(aN, aE, aD, dt);

    velN = kf3d.x[1];
    velE = kf3d.x[3];
    velD = kf3d.x[5];

    // 2. Control Failsafe y Piloto Automático por Waypoints (CADI_A)
    bool linkLost = (millis() - lastPacketReceivedMs > 1000);
    if (linkLost) {
      // Failsafe: Activar RTL si hay GPS
      currentMode = MODE_WAYPOINT;
      if (homeLat != 0.0f && homeLon != 0.0f) {
        waypointRoute[0].lat = homeLat;
        waypointRoute[0].lon = homeLon;
        waypointRoute[0].alt =
            15.0f;                 // Altitud RTL segura para 120mm quadcopter
        waypointRoute[0].mode = 1; // Waypoint
        waypointCount = 1;
        waypointIndex = 0;
        routeLoop = false;
      } else {
        // Nivelar y descender suavemente
        noInterrupts();
        setpoint_roll = 0.0f;
        setpoint_pitch = 0.0f;
        setpoint_throttle = 1200; // Descenso lento
        setpoint_yaw = 0.0f;
        interrupts();
        goto endLoop;
      }
    }

    if (currentMode == MODE_WAYPOINT || currentMode == MODE_ORBIT) {
      estimateWind();

      // Geocerca (1800 metros)
      if (homeLat != 0.0f && homeLon != 0.0f) {
        Vector3D localPos = GPSToLocal(latitude, longitude, altitudeM);
        float hDist = sqrtf(localPos.x * localPos.x + localPos.y * localPos.y);
        if (hDist > 1800.0f) {
          currentMode = MODE_WAYPOINT;
          waypointRoute[0].lat = homeLat;
          waypointRoute[0].lon = homeLon;
          waypointRoute[0].alt = 15.0f;
          waypointRoute[0].mode = 1;
          waypointCount = 1;
          waypointIndex = 0;
          routeLoop = false;
        }
      }

      // Ejecutar Algoritmo de Guía
      if (currentMode == MODE_ORBIT) {
        runOrbitNavigation();
      } else {
        runL1Navigation();
      }

      applySafetyGuards();
    }

  endLoop:
    // 3. Comunicaciones
    handle_radio_comms();
    send_telemetry_packet();
  }

  // 4. Almacenamiento Blackbox
  handle_flash_logging();
}
