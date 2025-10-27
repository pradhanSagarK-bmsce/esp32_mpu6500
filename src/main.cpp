#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "IMU.h"

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
IMU imu;

// Screen geometry
const int W = 128, H = 64, CX = W / 2, CY = H / 2;
const float SCALE = 55.0f;

// ------------- 3D VECTOR ----------------
struct Vec3 { float x, y, z; };

// --- Satellite model ---
static const Vec3 satelliteVerts[] = {
  // Hexagonal main body (front hex + back hex)
  {0.0f,  0.3f, -0.25f}, {0.26f,  0.15f, -0.25f}, {0.26f, -0.15f, -0.25f},
  {0.0f, -0.3f, -0.25f}, {-0.26f, -0.15f, -0.25f}, {-0.26f,  0.15f, -0.25f},
  {0.0f,  0.3f,  0.25f}, {0.26f,  0.15f,  0.25f}, {0.26f, -0.15f,  0.25f},
  {0.0f, -0.3f,  0.25f}, {-0.26f, -0.15f,  0.25f}, {-0.26f,  0.15f,  0.25f},

  // Left solar panel (outer frame)
  {-0.26f,  0.12f, -0.05f}, {-0.8f,  0.12f, -0.05f}, {-0.8f, -0.12f, -0.05f}, {-0.26f, -0.12f, -0.05f},
  {-0.26f,  0.12f,  0.05f}, {-0.8f,  0.12f,  0.05f}, {-0.8f, -0.12f,  0.05f}, {-0.26f, -0.12f,  0.05f},

  // Right solar panel
  {0.26f,  0.12f, -0.05f}, {0.8f,  0.12f, -0.05f}, {0.8f, -0.12f, -0.05f}, {0.26f, -0.12f, -0.05f},
  {0.26f,  0.12f,  0.05f}, {0.8f,  0.12f,  0.05f}, {0.8f, -0.12f,  0.05f}, {0.26f, -0.12f,  0.05f},

  // Antenna dish (centered front)
  {0.0f, 0.0f, -0.35f}, {0.0f, 0.1f, -0.3f}, {0.1f, 0.0f, -0.3f}, {0.0f, -0.1f, -0.3f}, {-0.1f, 0.0f, -0.3f},

  // Boom arm (sensor rod)
  {0.0f, 0.0f, 0.25f}, {0.0f, 0.0f, 0.55f}
};

static const uint8_t edges[][2] = {
  // Hexagonal body edges
  {0,1},{1,2},{2,3},{3,4},{4,5},{5,0},
  {6,7},{7,8},{8,9},{9,10},{10,11},{11,6},
  {0,6},{1,7},{2,8},{3,9},{4,10},{5,11},

  // Left panel frame
  {12,13},{13,14},{14,15},{15,12},{16,17},{17,18},{18,19},{19,16},{12,16},{13,17},{14,18},{15,19},

  // Right panel frame
  {20,21},{21,22},{22,23},{23,20},{24,25},{25,26},{26,27},{27,24},{20,24},{21,25},{22,26},{23,27},

  // Antenna dish
  {28,29},{28,30},{28,31},{28,32},{29,30},{30,31},{31,32},{32,29},

  // Boom arm
  {33,34}
};

// ------------- ROTATION + PROJECTION -------------
void project(const Vec3 &p, float roll, float pitch, float yaw, int &x, int &y) {
  float cr = cosf(roll), sr = sinf(roll);
  float cp = cosf(pitch), sp = sinf(pitch);
  float cy = cosf(yaw), sy = sinf(yaw);

  float r00 = cy * cp, r01 = cy * sp * sr - sy * cr, r02 = cy * sp * cr + sy * sr;
  float r10 = sy * cp, r11 = sy * sp * sr + cy * cr, r12 = sy * sp * cr - cy * sr;
  float r20 = -sp,     r21 = cp * sr,               r22 = cp * cr;

  float rx = r00 * p.x + r01 * p.y + r02 * p.z;
  float ry = r10 * p.x + r11 * p.y + r12 * p.z;
  float rz = r20 * p.x + r21 * p.y + r22 * p.z + 2.0f;

  float invZ = 1.0f / rz;
  x = CX + rx * SCALE * invZ;
  y = CY - ry * SCALE * invZ;
}

// ------------- DRAW 3D SATELLITE -------------
void drawSatellite(float roll, float pitch, float yaw) {
  const int nVerts = sizeof(satelliteVerts) / sizeof(satelliteVerts[0]);
  int vx[nVerts], vy[nVerts];
  for (int i = 0; i < nVerts; i++)
    project(satelliteVerts[i], roll, pitch, yaw, vx[i], vy[i]);

  for (auto &e : edges)
    u8g2.drawLine(vx[e[0]], vy[e[0]], vx[e[1]], vy[e[1]]);

  // subtle glowing core effect
  u8g2.drawDisc(CX, CY, 1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  u8g2.begin();
  imu.begin();
  imu.calibrateGyro();
  Serial.println("🚀 IMU Satellite View Ready");
}

void loop() {
  if (!imu.update()) return;
  ImuEuler e = imu.getEuler();

  u8g2.clearBuffer();
  drawSatellite(e.roll * DEG_TO_RAD, e.pitch * DEG_TO_RAD, e.yaw * DEG_TO_RAD);
  u8g2.sendBuffer();

  delay(15); // ~60 FPS
}
