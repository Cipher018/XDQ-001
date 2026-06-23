#include <mpu9250.h>
#include <Preferences.h> // [F5] NVS key storage
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <nRF24L01.h>

// ═══════════════════════════════════════════════════════
// TELEMETRY PACKET — OPTIMIZADO
// Tamaño: 4+4+2+2+2+2+2+2+2+2 = 24 bytes (límite nRF24: 32B)
// Conversiones para campos int16:
//   ángulos   × 10  → 0.1° resolución
//   gforce    × 100 → 0.01G resolución
//   velocidad × 100 → 0.01 m/s resolución
// ═══════════════════════════════════════════════════════
struct __attribute__((packed)) TelemetryPacket {
  // Lat/lon como int32 escalado x1e7 para evitar pérdida de precisión de float
  // y problemas de endianness. Rango: ±2147483647 → ±214.7483647°
  int32_t latitude;     // 4B — grados × 1e7  (ej: 4.1234567° → 41234567)
  int32_t longitude;    // 4B — grados × 1e7
  int16_t altitude;     // 2B — metros × 10  (−3276m a +3276m, 0.1m res)
  int16_t heading;      // 2B — grados × 10  (0 a 3599 → 0° a 359.9°)
  int16_t pitch;        // 2B — grados × 10  (−900 a +900 → −90° a +90°)
  int16_t roll;         // 2B — grados × 10  (−1800 a +1800)
  int16_t gforce;       // 2B — G × 100      (0 a 800 → 0G a 8G)
  int16_t velocityX;    // 2B — m/s × 100    (−5000 a +5000 → ±50 m/s)
  int16_t velocityY;    // 2B — m/s × 100
  int16_t velocityZ;    // 2B — m/s × 100
  uint8_t seq;          // 1B — Contador incremental para detectar pérdida
  uint8_t _pad[3];      // 3B — padding a 28 bytes (múltiplo de 4 para XXTEA)
};                      // Total: 28 bytes ✓

TelemetryPacket telemPkt;

// ═══════════════════════════════════════════════════════
// DUBINS AND VECTOR CLASS
// ═══════════════════════════════════════════════════════
class Vector3D {
public:
  float x, y, z;
  Vector3D(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}
  Vector3D operator-(const Vector3D &v) const { return Vector3D(x - v.x, y - v.y, z - v.z); }
  Vector3D operator+(const Vector3D &v) const { return Vector3D(x + v.x, y + v.y, z + v.z); }
  Vector3D operator*(float s) const { return Vector3D(x * s, y * s, z * s); }
  float magnitude() const { return sqrtf(x * x + y * y + z * z); }
  Vector3D normalize() const {
    float mag = magnitude();
    if (mag < 0.001f) return Vector3D(0, 0, 0);
    return Vector3D(x / mag, y / mag, z / mag);
  }
  float dot(const Vector3D &v) const { return x * v.x + y * v.y + z * v.z; }
  Vector3D cross(const Vector3D &v) const {
    return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }
};

// ── Global Mission State ──
const int MAX_WAYPOINTS = 16;
struct __attribute__((packed)) RouteWaypoint {
  float lat;
  float lon;
  float alt;
  uint8_t mode;      // 1=Waypoint, 2=Orbit
  uint8_t direction; // 0=CCW, 1=CW
  float radius;
};

RouteWaypoint waypointRoute[MAX_WAYPOINTS];
int waypointCount = 0;
int waypointIndex = 0;
bool routeLoop     = false;

enum NavigationMode { MODE_MANUAL = 0, MODE_WAYPOINT = 1, MODE_ORBIT = 2 };
NavigationMode currentMode = MODE_MANUAL;

// ═══════════════════════════════════════════════════════
// MAHONY AHRS — Estimación de Actitud con Cuaterniones
// Reemplaza 3 Kalmans 1D independientes. Inmune a gimbal lock.
// Ref: Mahony et al., 2008. Complementary filter on SO(3).
// ═══════════════════════════════════════════════════════
class MahonyAHRS {
public:
  float q0, q1, q2, q3;    // Cuaternión [w, x, y, z]
  float integralFBx, integralFBy, integralFBz;
  float twoKp, twoKi;

  MahonyAHRS(float kp = 2.0f, float ki = 0.005f)
    : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f)
    , integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f)
    , twoKp(2.0f * kp), twoKi(2.0f * ki) {}

  // Actualización con giroscopio + acelerómetro (en radianes)
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return;
    ax /= norm; ay /= norm; az /= norm;

    halfvx = q1*q3 - q0*q2;
    halfvy = q0*q1 + q2*q3;
    halfvz = q0*q0 - 0.5f + q3*q3;

    halfex = ay*halfvz - az*halfvy;
    halfey = az*halfvx - ax*halfvz;
    halfez = ax*halfvy - ay*halfvx;

    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt;
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }

    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
  }

  // Actualización con magnetómetro adicional
  void updateMag(float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float mx, float my, float mz, float dt) {
    float recipNorm;
    float q0q0=q0*q0, q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
    float q1q1=q1*q1, q1q2=q1*q2, q1q3=q1*q3;
    float q2q2=q2*q2, q2q3=q2*q3, q3q3=q3*q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) { updateIMU(gx,gy,gz,ax,ay,az,dt); return; }
    ax /= norm; ay /= norm; az /= norm;

    norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm < 0.001f) { updateIMU(gx,gy,gz,ax,ay,az,dt); return; }
    mx /= norm; my /= norm; mz /= norm;

    hx = 2.0f*(mx*(0.5f-q2q2-q3q3) + my*(q1q2-q0q3) + mz*(q1q3+q0q2));
    hy = 2.0f*(mx*(q1q2+q0q3) + my*(0.5f-q1q1-q3q3) + mz*(q2q3-q0q1));
    bx = sqrtf(hx*hx + hy*hy);
    bz = 2.0f*(mx*(q1q3-q0q2) + my*(q2q3+q0q1) + mz*(0.5f-q1q1-q2q2));

    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx*(0.5f-q2q2-q3q3) + bz*(q1q3-q0q2);
    halfwy = bx*(q1q2-q0q3) + bz*(q0q1+q2q3);
    halfwz = bx*(q0q2+q1q3) + bz*(0.5f-q1q1-q2q2);

    halfex = ay*halfvz - az*halfvy + (my*halfwz - mz*halfwy);
    halfey = az*halfvx - ax*halfvz + (mz*halfwx - mx*halfwz);
    halfez = ax*halfvy - ay*halfvx + (mx*halfwy - my*halfwx);

    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt;
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx; gy += integralFBy; gz += integralFBz;
    }
    gx += twoKp*halfex; gy += twoKp*halfey; gz += twoKp*halfez;
    gx *= 0.5f*dt; gy *= 0.5f*dt; gz *= 0.5f*dt;

    float qa=q0, qb=q1, qc=q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f / sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;
  }

  float getPitch() const {
    return asinf(2.0f*(q0*q2 - q3*q1)) * 57.2958f;
  }
  float getRoll() const {
    return atan2f(2.0f*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3) * 57.2958f;
  }
  float getYaw() const {
    float yaw = atan2f(2.0f*(q0*q3+q1*q2), q0*q0+q1*q1-q2*q2-q3*q3) * 57.2958f;
    if (yaw < 0.0f) yaw += 360.0f;
    return yaw;
  }
};

MahonyAHRS mahony(2.0f, 0.005f); // kp=2.0, ki=0.005

// ═══════════════════════════════════════════════════════
// KALMAN LINEAL 3D — Fusión GPS + IMU (Cinemática Traslacional)
// Estado: [posN, velN, posE, velE, posD, velD]
// Observación (GPS): posición + velocidad cada ~200ms
// Predicción (IMU): aceleración rotada a Earth-Frame cada 20ms
// ═══════════════════════════════════════════════════════
const float MAX_VEL_GPS = 60.0f;

class KinematicsKF3D {
public:
  // Estado: x[0..1]=posN/velN, x[2..3]=posE/velE, x[4..5]=posD/velD
  float x[6];
  // Covarianza (diagonal — modelo desacoplado por eje)
  float P[6];
  // Ruido de proceso (acelerómetro) y de observación (GPS)
  const float Qa = 0.5f;   // varianza aceleración IMU (m/s^2)^2
  const float Rp = 4.0f;   // varianza posición GPS (m)^2
  const float Rv = 0.25f;  // varianza velocidad GPS (m/s)^2

  KinematicsKF3D() {
    for(int i=0;i<6;i++){ x[i]=0.0f; P[i]=100.0f; }
  }

  // Predicción IMU: aceleración en Earth-Frame (m/s²), ya sin gravedad
  void predict(float aN, float aE, float aD, float dt) {
    float dt2 = dt * dt;
    // Eje Norte
    x[0] += x[1]*dt + 0.5f*aN*dt2;
    x[1] += aN*dt;
    P[0] += P[1]*dt*2.0f + Qa*dt2*dt2*0.25f;
    P[1] += Qa*dt2;
    // Eje Este
    x[2] += x[3]*dt + 0.5f*aE*dt2;
    x[3] += aE*dt;
    P[2] += P[3]*dt*2.0f + Qa*dt2*dt2*0.25f;
    P[3] += Qa*dt2;
    // Eje Abajo
    x[4] += x[5]*dt + 0.5f*aD*dt2;
    x[5] += aD*dt;
    P[4] += P[5]*dt*2.0f + Qa*dt2*dt2*0.25f;
    P[5] += Qa*dt2;
  }

  // Actualización GPS: posición y velocidad observadas
  void updateGPS(float posN, float posE, float posD,
                 float vN,   float vE,   float vD) {
    // Actualización posición Norte
    _updateScalar(0, posN, Rp);
    // Actualización velocidad Norte
    _updateScalar(1, vN,   Rv);
    // Actualización posición Este
    _updateScalar(2, posE, Rp);
    // Actualización velocidad Este
    _updateScalar(3, vE,   Rv);
    // Actualización posición Abajo
    _updateScalar(4, posD, Rp);
    // Actualización velocidad Abajo
    _updateScalar(5, vD,   Rv);
  }

  float velN() const { return x[1]; }
  float velE() const { return x[3]; }
  float velD() const { return x[5]; }
  float posN() const { return x[0]; }
  float posE() const { return x[2]; }
  float posD() const { return x[4]; }

private:
  void _updateScalar(int idx, float z, float R) {
    float K = P[idx] / (P[idx] + R);
    x[idx] += K * (z - x[idx]);
    P[idx] *= (1.0f - K);
  }
};

KinematicsKF3D kf3d;

// ═══════════════════════════════════════════════════════
// HARDWARE DEFINITIONS
// ═══════════════════════════════════════════════════════
Servo esc;
#define ESC_PIN 14

int servo1Pos = 90, servo2Pos = 90, servo3Pos = 90, servo4Pos = 90;
int servo5Pos = 90, servo6Pos = 90, servo7Pos = 90, servo8Pos = 90;

TinyGPSPlus gps;
bfs::Mpu9250  mpu;

#define GPS_SERIAL Serial2
#define GPS_RX 17
#define GPS_TX 16

#define CE_PIN  5
#define CSN_PIN 4
RF24 radio(CE_PIN, CSN_PIN);
const byte pipeRX[6] = "CMD01";
const byte pipeTX[6] = "TEL01";

// ═══════════════════════════════════════════════════════
// FHSS AND CHANNELS (Salto de frecuencia por software)
// ═══════════════════════════════════════════════════════
const uint8_t fhssChannels[8] = { 10, 25, 40, 55, 70, 85, 100, 115 };
uint8_t currentChannelIdx = 0;
unsigned long lastPacketReceivedMs = 0;

// ═══════════════════════════════════════════════════════
// SECURITY (XXTEA)
// ═══════════════════════════════════════════════════════
uint32_t sharedKey[4] = { 0x58444630, 0x30314B45, 0x595F3230, 0x32362121 }; // "XDF001KEY_2026!!" en hex
#define XXTEA_DELTA 0x9e3779b9
#define XXTEA_MX (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (sharedKey[(p&3)^e] ^ z)))

void btea(uint32_t *v, int n) {
  uint32_t y, z, sum;
  unsigned p, rounds, e;
  if (n > 1) {
    rounds = 6 + 52/n;
    sum = 0;
    z = v[n-1];
    do {
      sum += XXTEA_DELTA;
      e = (sum >> 2) & 3;
      for (p=0; p<n-1; p++) {
        y = v[p+1];
        z = v[p] += XXTEA_MX;
      }
      y = v[0];
      z = v[n-1] += XXTEA_MX;
    } while (--rounds);
  } else if (n < -1) {
    n = -n;
    rounds = 6 + 52/n;
    sum = rounds*XXTEA_DELTA;
    y = v[0];
    do {
      e = (sum >> 2) & 3;
      for (p=n-1; p>0; p--) {
        z = v[p-1];
        y = v[p] -= XXTEA_MX;
      }
      z = v[n-1];
      y = v[0] -= XXTEA_MX;
      sum -= XXTEA_DELTA;
    } while (--rounds);
  }
}

// ── Headers Seguros ──
const uint8_t MAGIC_CMD   = 0xAA;
const uint8_t MAGIC_TELEM = 0xBB;
const uint8_t MAGIC_WP    = 0xAC;

struct __attribute__((packed)) SecureCommand {
  uint8_t  magic;
  uint8_t  seq;
  uint16_t crc;
  int16_t  targetRoll;
  int16_t  targetPitch;
  int16_t  targetYaw;
  int16_t  targetThrottle;
  float    homeLat;
  float    homeLon;
  int16_t  declinationX10;
  uint8_t  navMode;        // 0=MANUAL, 1=WAYPOINT, 2=ORBIT
  uint8_t  cmdFlags;       // Bit 0 = Reset route index
}; // Total: 24 bytes (6 words)

struct __attribute__((packed)) SecureWaypointPacket {
  uint8_t  magic;          // MAGIC_WP = 0xAC
  uint8_t  seq;
  uint16_t crc;
  uint8_t  wpIndex;        // 0..15
  uint8_t  totalWps;
  uint8_t  routeLoop;      // 0=no, 1=yes
  uint8_t  mode;           // NavigationMode (0=MANUAL, 1=WAYPOINT, 2=ORBIT)
  RouteWaypoint wp;        // 18 bytes
  uint8_t  _pad[2];        // Pad to 28 bytes (multiple of 4)
}; // Total: 28 bytes (7 words)

struct __attribute__((packed)) SecureTelemetry {
  uint8_t  magic;
  uint8_t  seq;
  uint16_t crc;
  TelemetryPacket telem;
}; // 32 bytes

SecureCommand   secCmd;
SecureWaypointPacket secWpPkt;
SecureTelemetry secTelem;
uint8_t         telemSeq = 0;

// ── Target Angles and Flight State ──
float targetRoll     = 0.0f;
float targetPitch    = 0.0f;
float targetYaw      = 0.0f;
float targetThrottle = 0.0f;

float homeLat = 0.0f;
float homeLon = 0.0f;

uint16_t calculateRadioCRC16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = crc << 1;
    }
  }
  return crc;
}

unsigned long lastRFTime = 0;
bool imuActive = false;

// --- PCA9685 ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_CH_YAW    0
#define SERVO_CH_ROLL_L 1
#define SERVO_CH_ROLL_R 2
#define SERVO_CH_FLAP_L 3
#define SERVO_CH_FLAP_R 4
#define SERVO_CH_PITCH_L 5
#define SERVO_CH_PITCH_R 6
#define SERVO_PULSE_MIN 102
#define SERVO_PULSE_MAX 512
#define SERVO_FREQUENCY  50

// ═══════════════════════════════════════════════════════
// VARIABLES DE ESTADO (Compartidas / Volatile)
// ═══════════════════════════════════════════════════════
volatile float latitude   = 0.0f;
volatile float longitude  = 0.0f;
volatile float altitudeM  = 0.0f;
volatile float speedKmph  = 0.0f;

volatile float pitchDeg   = 0.0f;   // Kalman pitch (grados)
volatile float rollDeg    = 0.0f;   // Kalman roll  (grados)
volatile float yawDeg     = 0.0f;   // Kalman yaw   (grados, 0–360)
volatile float totalGForce= 1.0f;

volatile float velX = 0.0f;   // Este (m/s)
volatile float velY = 0.0f;   // Norte (m/s)
volatile float velZ = 0.0f;   // Arriba (m/s)

float lastLat  = 0.0f, lastLon  = 0.0f, lastAlt  = 0.0f;
unsigned long lastGPSVelTime = 0;
bool firstGPSFix = true;
unsigned long lastIMUTime = 0;
float magneticDeclinationDeg = -6.0f; 

// Wind vector variables
volatile float windX = 0.0f;  // East wind (m/s)
volatile float windY = 0.0f;  // North wind (m/s)
volatile float windZ = 0.0f;  // Vertical wind (m/s)

// FreeRTOS thread safety Mutex
portMUX_TYPE sharedStateMutex = portMUX_INITIALIZER_UNLOCKED;

// ═══════════════════════════════════════════════════════
// FLY-BY-WIRE & PID (LAZO INTERNO)
// ═══════════════════════════════════════════════════════
class SlewLimiter {
  float current;
  float maxRate; 
public:
  SlewLimiter(float rate) : current(0), maxRate(rate) {}
  float update(float target, float dt) {
    float delta = target - current;
    float maxDelta = maxRate * dt;
    if (delta > maxDelta) current += maxDelta;
    else if (delta < -maxDelta) current -= maxDelta;
    else current = target;
    return current;
  }
};

class PIDController {
public:
  float kp, ki, kd, kff;
  float integralMax;
  float integral;
  float lastMeasured;
  float lastDOut;
  bool firstRun;

  PIDController(float p, float i, float d, float ff, float iMax) : 
    kp(p), ki(i), kd(d), kff(ff), integralMax(iMax), integral(0), lastMeasured(0), lastDOut(0), firstRun(true) {}

  float compute(float setpoint, float measured, float dt) {
    if (firstRun) {
      lastMeasured = measured;
      firstRun = false;
    }
    
    float error = setpoint - measured;
    float pOut = kp * error;
    
    integral += error * dt;
    if (integral > integralMax) integral = integralMax;
    else if (integral < -integralMax) integral = -integralMax;
    float iOut = ki * integral;
    
    float dMeasured = (measured - lastMeasured) / dt;
    float rawDOut = -kd * dMeasured;
    // Low-Pass Filter on D-Term (alpha = 0.2)
    float dOut = lastDOut * 0.8f + rawDOut * 0.2f;
    lastDOut = dOut;
    lastMeasured = measured;

    // Feed-Forward
    float ffOut = kff * setpoint;
    
    return pOut + iOut + dOut + ffOut;
  }
};

SlewLimiter slewRoll(180.0f);   // Relajado para permitir mayor maniobrabilidad táctica
SlewLimiter slewPitch(180.0f);  // Relajado
PIDController pidRoll(1.2f, 0.1f, 0.2f, 0.5f, 20.0f);
PIDController pidPitch(1.5f, 0.1f, 0.2f, 0.6f, 20.0f);

// ═══════════════════════════════════════════════════════
// NAVIGATION CONSTANTS
// ═══════════════════════════════════════════════════════
const float WP_CAPTURE_NEAR   = 8.0f;    // metros — confirmación de llegada
const float WP_CAPTURE_FAR    = 15.0f;   // metros — zona de desaceleración
const float STALL_SPEED       = 6.0f;    // m/s — velocidad mínima de seguridad recto

// Forward declarations
void handleFhssHopping();
void runL1Navigation();
void runOrbitNavigation();
void applySafetyGuards();
void estimateWind();
void readGPS();
void readMPU();
void updateTelemetry();
void setServoAngle(uint8_t channel, int angle);
void pushTelemetryAck();

// ═══════════════════════════════════════════════════════
// FREERTOS TASKS
// ═══════════════════════════════════════════════════════
void taskStabilization(void *pvParameters);
void taskNavigationComms(void *pvParameters);

void setup() {
  Serial.begin(115200);
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  int mpuRetries = 10;
  while (!mpu.Begin() && mpuRetries > 0) {
    delay(100);
    mpuRetries--;
  }
  imuActive = (mpuRetries > 0);

  if (imuActive) {
    mpu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_8G);
    mpu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS);
    mpu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
    mpu.ConfigSrd(19);   // ~50Hz

    delay(1000);

    mpu.Read();
    float ax = mpu.accel_x_mps2();
    float ay = mpu.accel_y_mps2();
    float az = mpu.accel_z_mps2();
    float mx = mpu.mag_x_ut();
    float my = mpu.mag_y_ut();
    float mz = mpu.mag_z_ut();
    // Convergencia rápida del Mahony con 100 iteraciones de precalentamiento
    for (int k = 0; k < 100; k++) {
      mahony.updateMag(0,0,0, ax,ay,az, mx,my,mz, 0.01f);
    }
    lastIMUTime = millis();
  }

  if (!radio.begin()) { while (1); }
  // Protocolo Fire-and-Forget (UDP-style): sin ACKs ni retransmisiones.
  // Si se pierde un paquete, el siguiente con estado actualizado llegará en 20ms.
  // Esto elimina el jitter causado por el bloqueo del módulo durante reintentos.
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setPayloadSize(32);  // Tamaño fijo — más eficiente que dynamic payloads
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setChannel(fhssChannels[0]);
  radio.startListening();

  Preferences preferences;
  preferences.begin("pairing", false);
  if (preferences.isKey("shared_key")) {
    preferences.getBytes("shared_key", sharedKey, 16);
  }
  preferences.end();

  // Create thread-safe Tasks
  xTaskCreatePinnedToCore(
    taskStabilization, 
    "Stabilization", 
    4096, 
    NULL, 
    5, // Alta prioridad para lazo PID 50Hz
    NULL, 
    1  // Núcleo 1 (estabilización)
  );

  xTaskCreatePinnedToCore(
    taskNavigationComms, 
    "NavigationComms", 
    8192, 
    NULL, 
    2, // Prioridad normal para guiado/radio
    NULL, 
    0  // Núcleo 0 (cálculos vectoriales/comunicaciones)
  );

  lastRFTime = millis();
  lastPacketReceivedMs = millis();
}

void loop() {
  // Bucle principal vacío, FreeRTOS maneja las tareas en los dos núcleos
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ═══════════════════════════════════════════════════════
// CORE 1: ESTABILIZACIÓN RÁPIDA (50Hz)
// ═══════════════════════════════════════════════════════
void taskStabilization(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // Exactamente 20ms (50Hz)

  while (true) {
    float dt = 0.02f;

    if (imuActive) {
      readMPU();

      float rollLocal, pitchLocal;
      float targetRollLocal, targetPitchLocal, targetYawLocal, targetThrottleLocal;

      // Lectura y escritura atómica de variables compartidas
      portENTER_CRITICAL(&sharedStateMutex);
      targetRollLocal     = targetRoll;
      targetPitchLocal    = targetPitch;
      targetYawLocal      = targetYaw;
      targetThrottleLocal = targetThrottle;
      
      // Pasar actitud actual a navegación
      rollLocal           = rollDeg;
      pitchLocal          = pitchDeg;
      portEXIT_CRITICAL(&sharedStateMutex);

      // 1. Suavizar setpoints de usuario
      float smoothRoll = slewRoll.update(targetRollLocal, dt);
      float smoothPitch = slewPitch.update(targetPitchLocal, dt);

      // 2. Calcular PID
      float rollActuator = pidRoll.compute(smoothRoll, rollLocal, dt);
      float pitchActuator = pidPitch.compute(smoothPitch, pitchLocal, dt);
      float yawActuator = targetYawLocal; 

      // Mitigación de Torque Roll: rampa del acelerador para suavizar aceleraciones
      static float currentThrottle = 0.0f;
      float throttleDelta = targetThrottleLocal - currentThrottle;
      float maxThrottleDelta = 60.0f * dt; // max 60 unidades por segundo de rampa
      if (throttleDelta > maxThrottleDelta) currentThrottle += maxThrottleDelta;
      else if (throttleDelta < -maxThrottleDelta) currentThrottle -= maxThrottleDelta;
      else currentThrottle = targetThrottleLocal;

      // 3. Mezclador a Servos
      servo1Pos = (int)currentThrottle;
      servo2Pos = map(constrain(yawActuator, -40, 40),   -40, 40,  50, 130);
      servo3Pos = map(constrain(rollActuator, -45, 45),  -45, 45,  45, 135);
      servo4Pos = map(constrain(rollActuator, -45, 45),  -45, 45, 135,  45);  // invertido
      servo5Pos = 90;
      servo6Pos = 90;
      servo7Pos = map(constrain(pitchActuator, -30, 30), -30, 30,  60, 120);
      servo8Pos = map(constrain(pitchActuator, -30, 30), -30, 30,  60, 120);

      // 4. Aplicar al hardware
      esc.write(constrain(servo1Pos, 0, 180));
      setServoAngle(SERVO_CH_YAW,     servo2Pos);
      setServoAngle(SERVO_CH_ROLL_L,  servo3Pos);
      setServoAngle(SERVO_CH_ROLL_R,  servo4Pos);
      setServoAngle(SERVO_CH_FLAP_L,  servo5Pos);
      setServoAngle(SERVO_CH_FLAP_R,  servo6Pos);
      setServoAngle(SERVO_CH_PITCH_L, servo7Pos);
      setServoAngle(SERVO_CH_PITCH_R, servo8Pos);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ═══════════════════════════════════════════════════════
// CORE 0: NAVEGACIÓN Y COMUNICACIONES (10Hz)
// ═══════════════════════════════════════════════════════
static char setKeyLineBuf[32];
static uint8_t setKeyLineIdx = 0;

void handleSetKeyCommand(const char* line) {
  if (strncmp(line, "SET_KEY:", 8) != 0) return;
  const char* keyStr = line + 8;
  size_t keyLen = strlen(keyStr);
  uint8_t keyBytes[16] = {0};
  size_t copyLen = keyLen < 16 ? keyLen : 16;
  memcpy(keyBytes, keyStr, copyLen);

  Preferences prefs;
  prefs.begin("pairing", false);
  prefs.putBytes("shared_key", keyBytes, 16);
  prefs.end();

  memcpy(sharedKey, keyBytes, 16);
  Serial.println("[SEC] New key stored in NVS. Restarting...");
  delay(200);
  ESP.restart();
}

void parseSetKeySerial() {
  while (Serial.available()) {
    char ch = (char)Serial.peek();
    if ((uint8_t)ch == MAGIC_CMD || (uint8_t)ch == MAGIC_TELEM || (uint8_t)ch == MAGIC_WP) break;
    Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (setKeyLineIdx > 0) {
        setKeyLineBuf[setKeyLineIdx] = '\0';
        handleSetKeyCommand(setKeyLineBuf);
        setKeyLineIdx = 0;
      }
    } else if (setKeyLineIdx < 30) {
      setKeyLineBuf[setKeyLineIdx++] = ch;
    }
  }
}

void taskNavigationComms(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz (100ms)

  while (true) {
    parseSetKeySerial();
    readGPS();

    // ── 1. Salto de frecuencia FHSS por software ──
    handleFhssHopping();

    // ── 2. Recepción de Radio ──
    if (radio.available()) {
      uint8_t len = radio.getDynamicPayloadSize();
      if (len == sizeof(SecureCommand)) {
        radio.read(&secCmd, sizeof(SecureCommand));
        btea((uint32_t*)&secCmd, -6); // Descifrar

        uint16_t crc = calculateRadioCRC16((uint8_t*)&secCmd.targetRoll, 20); 
        if (secCmd.magic == MAGIC_CMD && secCmd.crc == crc) {
          static uint8_t lastSeq = 255;
          uint8_t seqDelta = (uint8_t)(secCmd.seq - lastSeq);
          if (seqDelta > 0 && seqDelta <= 128) {
            lastSeq = secCmd.seq;
            lastRFTime = millis();
            lastPacketReceivedMs = millis();

            // Modo de navegación solicitado por GCS
            currentMode = (NavigationMode)secCmd.navMode;

            // Resetear ruta si se indica flag
            if (secCmd.cmdFlags & 0x01) {
              waypointIndex = 0;
            }

            if (currentMode == MODE_MANUAL) {
              // Modo directo (Joystick manual override)
              portENTER_CRITICAL(&sharedStateMutex);
              targetRoll     = secCmd.targetRoll;
              targetPitch    = secCmd.targetPitch;
              targetYaw      = secCmd.targetYaw;
              targetThrottle = secCmd.targetThrottle;
              portEXIT_CRITICAL(&sharedStateMutex);
            }

            magneticDeclinationDeg = secCmd.declinationX10 / 10.0f;
            if (secCmd.homeLat != 0.0f && secCmd.homeLon != 0.0f) {
              homeLat = secCmd.homeLat;
              homeLon = secCmd.homeLon;
            }

            pushTelemetryAck();
          }
        }
      } 
      else if (len == sizeof(SecureWaypointPacket)) {
        radio.read(&secWpPkt, sizeof(SecureWaypointPacket));
        btea((uint32_t*)&secWpPkt, -7); // Descifrar 7 palabras (28 bytes)

        uint16_t crc = calculateRadioCRC16((uint8_t*)&secWpPkt.wpIndex, 24);
        if (secWpPkt.magic == MAGIC_WP && secWpPkt.crc == crc) {
          lastPacketReceivedMs = millis();
          lastRFTime = millis();

          uint8_t idx = secWpPkt.wpIndex;
          if (idx < MAX_WAYPOINTS) {
            waypointRoute[idx] = secWpPkt.wp;
            waypointCount      = secWpPkt.totalWps;
            routeLoop          = (secWpPkt.routeLoop == 1);
            currentMode        = (NavigationMode)secWpPkt.mode;
            
            if (idx == 0) {
              waypointIndex = 0;
            }
          }
          pushTelemetryAck();
        }
      }
      else {
        uint8_t dummy[32];
        radio.read(&dummy, len);
      }
    }

    // ── 3. FAILSAFE / AUTOPILOT CONTROL ──
    bool linkLost = (millis() - lastRFTime > 1000);
    
    if (linkLost) {
      // Pérdida de señal: Invocar RTL autónomo
      currentMode = MODE_WAYPOINT;
      
      // Si home está establecido, crear waypoint temporal hacia home
      if (homeLat != 0.0f && homeLon != 0.0f) {
        waypointRoute[0].lat = homeLat;
        waypointRoute[0].lon = homeLon;
        waypointRoute[0].alt = 40.0f; // 40m altitud segura RTL
        waypointRoute[0].mode = 1; // Waypoint
        waypointCount = 1;
        waypointIndex = 0;
        routeLoop = false;
      } else {
        // Fallback: Senda de planeo nivelada si no hay GPS/Home
        portENTER_CRITICAL(&sharedStateMutex);
        targetRoll = 0;
        targetPitch = 3.0f;
        targetThrottle = 0;
        targetYaw = 0;
        portEXIT_CRITICAL(&sharedStateMutex);
        goto endLoop;
      }
    }

    // ── 4. CÁLCULO DE NAVEGACIÓN VECTORIAL COMPLETA ONBOARD ──
    if (currentMode == MODE_WAYPOINT || currentMode == MODE_ORBIT) {
      estimateWind();

      // ── Onboard Geofence Protection ──
      if (homeLat != 0.0f && homeLon != 0.0f) {
        Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
        float hDist = sqrtf(pos.x * pos.x + pos.y * pos.y);
        if (hDist > 1800.0f) {
          // Si nos salimos de los 1800m de geocerca, forzar RTL automático hacia home
          currentMode = MODE_WAYPOINT;
          waypointRoute[0].lat = homeLat;
          waypointRoute[0].lon = homeLon;
          waypointRoute[0].alt = 40.0f; // 40m altitud segura RTL
          waypointRoute[0].mode = 1; // Waypoint
          waypointCount = 1;
          waypointIndex = 0;
          routeLoop = false;
        }
      }

      RouteWaypoint currentWP = waypointRoute[waypointIndex];
      if (currentWP.mode == 2 || currentMode == MODE_ORBIT) {
        runOrbitNavigation();
      } else {
        runL1Navigation();
      }

      // Aplicar protecciones de Stall dinámico y limitador de Gs estructurales
      applySafetyGuards();
    }

    endLoop:
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ═══════════════════════════════════════════════════════
// ESTIMADOR HÍBRIDO DE VIENTO SIN TUBO DE PITOT
// ═══════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════
// ESTIMADOR DE VIENTO — Triángulo Estocástico en Virajes
// Solo corre cuando |roll| > 10 grados (viraje): en ese momento
// el GPS mide la trayectoria real sobre el suelo, y el Mahony da
// la orientación de la nariz. La diferencia es el viento.
// Sin datos en línea recta (inobservable sin pitot): se mantiene
// la última estimación filtrada.
// ═══════════════════════════════════════════════════════
void estimateWind() {
  float rollLocal, pitchLocal, yawLocal;
  portENTER_CRITICAL(&sharedStateMutex);
  rollLocal  = rollDeg;
  pitchLocal = pitchDeg;
  yawLocal   = yawDeg;
  portEXIT_CRITICAL(&sharedStateMutex);

  // Solo actualizar estimación de viento durante virajes (roll > 10 grados)
  // En línea recta, el viento es inobservable sin tubo de Pitot.
  if (fabsf(rollLocal) < 10.0f) return;

  // Velocidad aerodinámica estimada: conservadora basada en el GPS
  float groundSpeed = sqrtf(velX*velX + velY*velY);
  if (groundSpeed < 3.0f) return;  // GPS no confiable a baja velocidad

  float pitchRad = pitchLocal * 0.0174533f;
  float yawRad   = yawLocal   * 0.0174533f;

  // Velocidad aerodínamica = velocidad de suelo + viento (modelo triángulo)
  // Dirección de la nariz del avión (body X en Earth Frame)
  float noseDirX = cosf(pitchRad) * sinf(yawRad);  // Este
  float noseDirY = cosf(pitchRad) * cosf(yawRad);  // Norte

  // Diferencia entre hacia dónde apunta la nariz y hacia dónde se mueve
  // sobre el suelo = viento instantaneo (sin escala de airspeed)
  float rawWindX = velX - groundSpeed * noseDirX;
  float rawWindY = velY - groundSpeed * noseDirY;

  // Filtro exponencial de baja frecuencia (alpha = 0.05 — promedia ~20 muestras)
  windX = windX * 0.95f + rawWindX * 0.05f;
  windY = windY * 0.95f + rawWindY * 0.05f;
  windZ = windZ * 0.98f + 0.0f;  // Viento vertical no estimable, mantener filtrado
}

// ═══════════════════════════════════════════════════════
// NAVEGACIÓN LATERAL L1 ADAPTATIVA Y TECS
// ═══════════════════════════════════════════════════════
Vector3D GPSToLocal(float lat, float lon, float alt) {
  if (homeLat == 0.0f || homeLon == 0.0f)
    return Vector3D(0, 0, 0);

  float dLat = (lat - homeLat) * 111320.0f;
  float dLon = (lon - homeLon) * 111320.0f * cosf(lat * 3.14159f / 180.0f);
  float dAlt = alt; // Altitud relativa

  return Vector3D(dLon, dLat, dAlt);
}

void runL1Navigation() {
  if (waypointCount == 0) {
    portENTER_CRITICAL(&sharedStateMutex);
    targetRoll = 0;
    targetPitch = 3.0f;
    targetThrottle = 90;
    portEXIT_CRITICAL(&sharedStateMutex);
    return;
  }

  Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
  Vector3D velocity(velX, velY, velZ);
  float speed = max(velocity.magnitude(), 2.0f);

  RouteWaypoint currentWP = waypointRoute[waypointIndex];
  Vector3D B = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);

  // Zonas de captura: fly-by dinámico
  float distToWP = (B - pos).magnitude();
  if (distToWP < WP_CAPTURE_NEAR) {
    waypointIndex++;
    if (waypointIndex >= waypointCount) {
      if (routeLoop) {
        waypointIndex = 0;
      } else {
        waypointIndex = waypointCount - 1;
        // Si no hay bucle, órbita de loiter en el último WP
        currentWP.mode = 2;
        currentWP.radius = 30.0f;
        currentWP.direction = 1;
      }
    }
    currentWP = waypointRoute[waypointIndex];
    B = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);
  }

  // Segmento AB L1
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

  // L1 distancia adaptativa
  float L1_dist = 3.0f * speed;
  L1_dist = constrain(L1_dist, 10.0f, 60.0f);

  float projDist = AP.dot(AB_norm);
  float targetDistOnSeg = projDist + sqrtf(max(0.0f, L1_dist*L1_dist - crossTrackErr*crossTrackErr));
  targetDistOnSeg = constrain(targetDistOnSeg, 0.0f, segLen);

  Vector3D targetL1 = A + AB_norm * targetDistOnSeg;
  Vector3D toTargetL1 = targetL1 - pos;

  float yawLocal;
  portENTER_CRITICAL(&sharedStateMutex);
  yawLocal = yawDeg;
  portEXIT_CRITICAL(&sharedStateMutex);

  float yawRad = yawLocal * 0.0174533f;
  Vector3D headingDir(sinf(yawRad), cosf(yawRad), 0);

  float dot = headingDir.dot(toTargetL1.normalize());
  dot = constrain(dot, -1.0f, 1.0f);
  float eta = acosf(dot);
  if (toTargetL1.x * headingDir.y - toTargetL1.y * headingDir.x < 0) {
    eta = -eta;
  }

  float a_lat = 2.0f * speed * speed / L1_dist * sinf(eta);
  float rollL1 = atanf(a_lat / 9.81f) * 57.2958f;

  float rollOut = constrain(rollL1, -35.0f, 35.0f);

  // Compensar viento cruzado feed-forward en roll
  float crosswind = -windX * cosf(yawRad) + windY * sinf(yawRad);
  rollOut += constrain(crosswind * 1.5f, -10.0f, 10.0f);
  rollOut = constrain(rollOut, -35.0f, 35.0f);

  // TECS (Total Energy Control System) para Pitch y Throttle
  float altErr = currentWP.alt - altitudeM;
  float speedErr = 12.0f - speed; // Velocidad objetivo 12 m/s

  float energyTotalErr = altErr * 0.1f + speedErr * 0.2f;
  float energyDistErr  = altErr * 0.1f - speedErr * 0.2f;

  float throttleTECS = 100.0f + energyTotalErr * 45.0f;
  float pitchTECS    = energyDistErr * 15.0f;

  portENTER_CRITICAL(&sharedStateMutex);
  targetRoll     = rollOut;
  targetPitch    = constrain(pitchTECS, -10.0f, 10.0f);
  targetThrottle = constrain((int)throttleTECS, 50, 160);
  targetYaw      = 0;
  portEXIT_CRITICAL(&sharedStateMutex);
}

// ═══════════════════════════════════════════════════════
// NAVEGACIÓN ORBITAL
// ═══════════════════════════════════════════════════════
void runOrbitNavigation() {
  Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
  Vector3D velocity(velX, velY, velZ);
  float speed = max(velocity.magnitude(), 2.0f);

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
  float rollCorr = (radiusErr * 0.6f + radiusDeriv * 0.2f) * (cw ? 1.0f : -1.0f);
  float rollOrbit = baseRoll + rollCorr;

  float altErr = currentWP.alt - altitudeM;
  float pitchOrbit = constrain(altErr * 0.5f, -10.0f, 10.0f);

  float rollLocal;
  portENTER_CRITICAL(&sharedStateMutex);
  rollLocal = rollDeg;
  portEXIT_CRITICAL(&sharedStateMutex);

  float cosRoll = cosf(rollLocal * 0.0174533f);
  if (cosRoll < 0.2f) cosRoll = 0.2f;
  float throttleCoordinated = 100.0f / cosRoll;

  portENTER_CRITICAL(&sharedStateMutex);
  targetRoll     = constrain(rollOrbit, -35.0f, 35.0f);
  targetPitch    = pitchOrbit;
  targetThrottle = constrain((int)throttleCoordinated, 70, 150);
  targetYaw      = 0;
  portEXIT_CRITICAL(&sharedStateMutex);
}

// ═══════════════════════════════════════════════════════
// MEDIDAS DE SEGURIDAD (Stall Guard, G-Limiter)
// ═══════════════════════════════════════════════════════
void applySafetyGuards() {
  float rollLocal, gforceLocal, trLocal, tpLocal;
  portENTER_CRITICAL(&sharedStateMutex);
  rollLocal   = rollDeg;
  gforceLocal = totalGForce;
  trLocal     = targetRoll;
  tpLocal     = targetPitch;
  portEXIT_CRITICAL(&sharedStateMutex);

  // 1. Dynamic Stall Guard
  float cosRoll = cosf(rollLocal * 0.0174533f);
  if (cosRoll < 0.2f) cosRoll = 0.2f;
  float v_stall_din = STALL_SPEED / sqrtf(cosRoll);

  float speed = sqrtf(velX*velX + velY*velY + velZ*velZ);
  if (speed < v_stall_din) {
    portENTER_CRITICAL(&sharedStateMutex);
    targetRoll     = 0.0f;
    targetPitch    = -5.0f;
    targetThrottle = 180;
    targetYaw      = 0.0f;
    portEXIT_CRITICAL(&sharedStateMutex);
    return;
  }

  // 2. G-Limiter SUAVE (Soft-Blending continuo)
  // Atenuación proporcional entre 2.0G y 3.5G para evitar escalones bruscos
  // que causan oscilaciones de límite (limit-cycling chattering).
  const float G_SOFT_START = 2.0f;
  const float G_SOFT_END   = 3.5f;
  if (gforceLocal > G_SOFT_START) {
    float attenuation = 1.0f - constrain(
      (gforceLocal - G_SOFT_START) / (G_SOFT_END - G_SOFT_START),
      0.0f, 0.7f  // max 70% de reducción para no perder autoridad total
    );
    portENTER_CRITICAL(&sharedStateMutex);
    targetRoll  = trLocal * attenuation;
    targetPitch = tpLocal * attenuation;
    portEXIT_CRITICAL(&sharedStateMutex);
  }
}

// ═══════════════════════════════════════════════════════
// FHSS SOFTWARE CHANNEL HOPPING
// ═══════════════════════════════════════════════════════
void handleFhssHopping() {
  if (millis() - lastPacketReceivedMs > 1000) {
    // Si perdimos el enlace por más de 1 segundo, saltar de canal cada 100ms buscando la base
    static unsigned long lastHopMs = 0;
    if (millis() - lastHopMs > 100) {
      currentChannelIdx = (currentChannelIdx + 1) % 8;
      radio.setChannel(fhssChannels[currentChannelIdx]);
      lastHopMs = millis();
    }
  }
}

// ═══════════════════════════════════════════════════════
// READ GPS
// ═══════════════════════════════════════════════════════
void readGPS() {
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    if (!gps.encode(c)) continue;

    if (gps.location.isValid()) {
      float lat = gps.location.lat();
      float lon = gps.location.lng();
      float alt = gps.altitude.isValid() ? gps.altitude.meters() : altitudeM;

      if (firstGPSFix) {
        lastLat = lat; lastLon = lon; lastAlt = alt;
        lastGPSVelTime = millis();
        firstGPSFix = false;
        // Inicializar posición del KF3D en cero (marco local desde home)
        // La posición se restablece cuando se recibe el Home del GCS
      } else {
        unsigned long now = millis();
        float dt = (now - lastGPSVelTime) / 1000.0f;

        if (dt > 0.05f) {
          float dNorth = (lat - lastLat) * 111320.0f;
          float dEast  = (lon - lastLon) * 111320.0f * cosf(lat * 0.0174533f);
          float dUp    = alt - lastAlt;
          float gpsVx  = dEast  / dt;  // Este
          float gpsVy  = dNorth / dt;  // Norte
          float gpsVz  = dUp    / dt;  // Arriba

          // Sanity check
          if (fabsf(gpsVx) < MAX_VEL_GPS &&
              fabsf(gpsVy) < MAX_VEL_GPS &&
              fabsf(gpsVz) < MAX_VEL_GPS) {

            // Marco local del KF3D: Norte=Y, Este=X, Abajo=-Z
            float posN_local = dNorth; // acumulativo
            float posE_local = dEast;
            float posD_local = -dUp;

            // Actualizar KF3D con observación GPS (posición y velocidad)
            kf3d.updateGPS(
              kf3d.posN() + dNorth,  // posición Norte acumulada
              kf3d.posE() + dEast,   // posición Este acumulada
              kf3d.posD() - dUp,     // posición Abajo acumulada
              gpsVy,   // velocidad Norte
              gpsVx,   // velocidad Este
             -gpsVz    // velocidad Abajo (positivo hacia abajo)
            );

            // Publicar velocidades del KF3D (a las variables globales)
            portENTER_CRITICAL(&sharedStateMutex);
            velX = kf3d.velE();    // Este  -> X
            velY = kf3d.velN();    // Norte -> Y
            velZ = -kf3d.velD();   // Abajo -> Z (positivo hacia arriba)
            portEXIT_CRITICAL(&sharedStateMutex);
          }

          lastLat = lat; lastLon = lon; lastAlt = alt;
          lastGPSVelTime = now;
        }
      }

      latitude  = lat;
      longitude = lon;
      altitudeM = alt;
    }

    if (gps.speed.isValid()) {
      speedKmph = gps.speed.kmph();
    }
  }
}

// ═══════════════════════════════════════════════════════
// READ MPU — Mahony AHRS + Kalman Lineal 3D (50Hz)
// ═══════════════════════════════════════════════════════
void readMPU() {
  if (!mpu.Read()) return;

  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  lastIMUTime = now;

  // --- Lectura bruta de sensores ---
  float ax = mpu.accel_x_mps2();
  float ay = mpu.accel_y_mps2();
  float az = mpu.accel_z_mps2();
  float gx = mpu.gyro_x_radps();
  float gy = mpu.gyro_y_radps();
  float gz = mpu.gyro_z_radps();
  float mx = mpu.mag_x_ut();
  float my = mpu.mag_y_ut();
  float mz = mpu.mag_z_ut();

  totalGForce = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;

  // --- 1. Actualizar Mahony AHRS (Actitud) ---
  // Fusiona giroscopio + acelerómetro + magnetómetro en cuaternión
  mahony.updateMag(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

  portENTER_CRITICAL(&sharedStateMutex);
  pitchDeg = mahony.getPitch();
  rollDeg  = mahony.getRoll();
  yawDeg   = mahony.getYaw() + magneticDeclinationDeg;
  if (yawDeg < 0.0f)   yawDeg += 360.0f;
  if (yawDeg >= 360.0f) yawDeg -= 360.0f;
  portEXIT_CRITICAL(&sharedStateMutex);

  // --- 2. Predicción KF3D: rotar aceleración IMU a Earth-Frame ---
  // Usamos el cuaternión de Mahony para evitar los costes de trig adicional
  float q0=mahony.q0, q1=mahony.q1, q2=mahony.q2, q3=mahony.q3;

  // Rotación body->earth: a_earth = R * a_body
  float axE = (1-2*(q2*q2+q3*q3))*ax + 2*(q1*q2-q0*q3)*ay + 2*(q1*q3+q0*q2)*az;
  float ayE = 2*(q1*q2+q0*q3)*ax + (1-2*(q1*q1+q3*q3))*ay + 2*(q2*q3-q0*q1)*az;
  float azE = 2*(q1*q3-q0*q2)*ax + 2*(q2*q3+q0*q1)*ay + (1-2*(q1*q1+q2*q2))*az;

  // Restar gravedad (Earth-Frame Z apunta hacia arriba => +9.81 en azE)
  float aN =  ayE;          // Norte
  float aE =  axE;          // Este
  float aD = -(azE - 9.81f); // Abajo (positivo hacia abajo)

  // Predicción inercial del KF3D a 50Hz
  kf3d.predict(aN, aE, aD, dt);
}

void updateTelemetry() {
  // lat/lon como int32 escalado x1e7 — sin pérdida de precisión
  telemPkt.latitude  = (int32_t)(latitude  * 1e7f);
  telemPkt.longitude = (int32_t)(longitude * 1e7f);
  telemPkt.altitude  = (int16_t)constrain(altitudeM   * 10.0f,  -32767, 32767);
  telemPkt.heading   = (int16_t)constrain(yawDeg      * 10.0f,       0, 35990);
  telemPkt.pitch     = (int16_t)constrain(pitchDeg    * 10.0f,   -9000,  9000);
  telemPkt.roll      = (int16_t)constrain(rollDeg     * 10.0f,  -18000, 18000);
  telemPkt.gforce    = (int16_t)constrain(totalGForce * 100.0f,      0,  3200);
  // Velocidades del KF3D fusionado (más suave y preciso que el GPS crudo)
  telemPkt.velocityX = (int16_t)constrain(velX        * 100.0f,  -5000,  5000);
  telemPkt.velocityY = (int16_t)constrain(velY        * 100.0f,  -5000,  5000);
  telemPkt.velocityZ = (int16_t)constrain(velZ        * 100.0f,  -5000,  5000);
}

void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(channel, 0, pulse);
}

void pushTelemetryAck() {
  telemPkt.seq++;
  updateTelemetry();
  secTelem.magic = MAGIC_TELEM;
  secTelem.seq   = telemSeq++;
  secTelem.telem = telemPkt;
  secTelem.crc   = calculateRadioCRC16((uint8_t*)&secTelem.telem, sizeof(TelemetryPacket));
  btea((uint32_t*)&secTelem, 8);
  // Fire-and-Forget: escribir el paquete de telemetría en la pipe de TX
  // sin esperar ACK. El GCS recibe el siguiente paquete si éste se pierde.
  radio.stopListening();
  radio.write(&secTelem, sizeof(SecureTelemetry));
  radio.startListening();
}
