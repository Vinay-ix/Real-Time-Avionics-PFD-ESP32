
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <math.h>

// ---------------- Display/I2C ----------------
#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define SDA_PIN        21
#define SCL_PIN        22

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ---------------- IMUs ----------------
Adafruit_MPU6050 mpu;
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

// ---------------- Baro ----------------
Adafruit_BMP280 bmp;
float QNH_hPa = 1013.25f; // sea level pressure

// ---------------- State ----------------
float filteredRoll = 0, filteredPitch = 0;
float adxlYawOffsetRad = 0.0f;

bool  mpuFail = false;
int   mismatchCount = 0;

bool  bmpOK = false;
float altitude_m = 0.0f;
float altFiltered_m = 0.0f;
float vsi_mps = 0.0f;
float vsiFiltered_mps = 0.0f;

unsigned long lastTime   = 0;
unsigned long lastAltMs  = 0;
float         lastAlt_m  = 0.0f;

// Simple heading estimate from gyro-Z (deg)
float heading_deg = 0.0f;

// ---------------------------------------------------------------
// SMALL MATH HELPERS
// ---------------------------------------------------------------
static inline float deg(float rad){ return rad * 57.2957795f; }
static inline float rad(float d){ return d * 0.0174532925f; }

float getRoll(float ax, float ay, float az)  { return deg(atan2f(ay, az)); }
float getPitch(float ax, float ay, float az) { return deg(atanf(-ax / sqrtf(ay*ay + az*az))); }

void rotateXY(float x, float y, float theta, float &xr, float &yr) {
  float c = cosf(theta), s = sinf(theta);
  xr = c*x - s*y;
  yr = s*x + c*y;
}

bool checkMPUalive() {
  if (mpuFail) return false;
  Wire.beginTransmission(0x68);
  return (Wire.endTransmission() == 0);
}

// ---------------------------------------------------------------
// WELCOME SCREEN
// ---------------------------------------------------------------
void showWelcomeScreen(bool mpuOK, bool adxlOK, bool bmpOK) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(8, 6);   display.println(" AIRCRAFT ATTITUDE SYS");
  display.setCursor(5, 22);  display.print("MPU6050 : "); display.println(mpuOK ? "OK" : "FAIL");
  display.setCursor(5, 32);  display.print("ADXL345: ");  display.println(adxlOK ? "OK" : "FAIL");
  display.setCursor(5, 42);  display.print("BMP280 : ");  display.println(bmpOK ? "OK" : "FAIL");
  display.setCursor(18, 54); display.println("Initializing...");
  display.display();
  delay(1200);
}

// ---------------------------------------------------------------
// BANK ANGLE MARKERS
// ---------------------------------------------------------------
void drawBankMarkers(float rollDeg) {
  int cx = OLED_WIDTH / 2;
  int topY = 4;
  float rad = 28;

  int marks[] = {10, 20, 30, 45};
  float rollRad = rollDeg * 0.0174533f;

  for (int i = 0; i < 4; i++) {
    float m = marks[i] * 0.0174533f;

    float x1 = cx + rad * sinf(m + rollRad);
    float y1 = topY + rad * cosf(m + rollRad);
    float x2 = cx + (rad - 5) * sinf(m + rollRad);
    float y2 = topY + (rad - 5) * cosf(m + rollRad);
    display.drawLine((int)x1, (int)y1, (int)x2, (int)y2, SSD1306_WHITE);

    x1 = cx - rad * sinf(m - rollRad);
    y1 = topY + rad * cosf(m - rollRad);
    x2 = cx - (rad - 5) * sinf(m - rollRad);
    y2 = topY + (rad - 5) * cosf(m - rollRad);
    display.drawLine((int)x1, (int)y1, (int)x2, (int)y2, SSD1306_WHITE);
  }
}

// ---------------------------------------------------------------
// ARTIFICIAL HORIZON
// ---------------------------------------------------------------
void drawHorizon(float roll, float pitch) {
  int cx = OLED_WIDTH / 2;
  int cy = OLED_HEIGHT / 2;

  float pitchOffset = pitch * 2.0f;
  float ang = roll * 0.0174533f;
  int L = 200;

  int x1 = (int)(cx - L * cosf(ang));
  int y1 = (int)(cy - L * sinf(ang) + pitchOffset);
  int x2 = (int)(cx + L * cosf(ang));
  int y2 = (int)(cy + L * sinf(ang) + pitchOffset);
  display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);

  display.drawLine(cx - 10, cy, cx + 10, cy, SSD1306_WHITE);
  display.drawLine(cx, cy, cx, cy + 5, SSD1306_WHITE);
}

// ---------------------------------------------------------------
// ALTITUDE / VSI PANEL
// ---------------------------------------------------------------
void drawAltVSI() {
  const int x = 84;
  display.drawRect(x - 2, 0, OLED_WIDTH - (x - 2), OLED_HEIGHT, SSD1306_WHITE);

  display.setTextSize(1);

  display.setCursor(x, 4);
  display.print("ALT:");
  display.setCursor(x + 28, 4);
  display.print((int)roundf(altFiltered_m));

  char arrow = 'o';
  if (vsiFiltered_mps > 0.1f) arrow = '^';
  else if (vsiFiltered_mps < -0.1f) arrow = 'v';

  float vsi_fpm = vsiFiltered_mps * 196.85f;

  display.setCursor(x, 18); display.print("VSI:");
  display.setCursor(x + 28, 18); display.print(arrow);

  display.setCursor(x, 30); display.print(vsiFiltered_mps, 2); display.print(" m/s");
  display.setCursor(x, 42); display.print((int)roundf(vsi_fpm)); display.print(" fpm");
}

// ---------------------------------------------------------------
// FAIL MESSAGE
// ---------------------------------------------------------------
void drawFailMsg() {
  display.setCursor(0, 54);
  display.print("IMU FAIL -> ADXL");
}

// ---------------------------------------------------------------
// ADXL YAW CALIBRATION
// ---------------------------------------------------------------
void calibrateADXLyaw() {
  sensors_event_t am, gm, tm, aa;

  display.clearDisplay();
  display.setCursor(10, 28);
  display.print("Calibrating...");
  display.display();
  delay(800);

  float mx = 0, my = 0, ax = 0, ay = 0;

  for (int i = 0; i < 200; i++) {
    mpu.getEvent(&am, &gm, &tm);
    adxl.getEvent(&aa);
    mx += am.acceleration.x;
    my += am.acceleration.y;
    ax += aa.acceleration.x;
    ay += aa.acceleration.y;
    delay(5);
  }

  mx /= 200;  my /= 200;
  ax /= 200;  ay /= 200;

  adxlYawOffsetRad = atan2f(ay, ax) - atan2f(my, mx);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("dYaw=");
  display.print(adxlYawOffsetRad * 57.2958f, 1);
  display.print(" deg");
  display.display();
  delay(500);
}

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  if (!mpu.begin()) mpuFail = true;
  if (!adxl.begin()) mpuFail = true;

  bmpOK = bmp.begin(0x76);
  if (bmpOK) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X4,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_125);
  } else {
    // avoid NaNs if sensor not present
    altitude_m = altFiltered_m = 0.0f;
  }

  showWelcomeScreen(!mpuFail, true, bmpOK);
  calibrateADXLyaw();

  lastTime  = millis();
  lastAltMs = millis();

  if (bmpOK) {
    float a0 = bmp.readAltitude(QNH_hPa);
    if (!isnan(a0)) {
      altitude_m    = a0;
      altFiltered_m = a0;
      lastAlt_m     = a0;
    }
  }
}

// ---------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------
void loop() {
  display.clearDisplay();

  if (!checkMPUalive()) mpuFail = true;

  // --- Read ADXL (backup) ---
  sensors_event_t adxlEv;
  adxl.getEvent(&adxlEv);
  float axr, ayr;
  rotateXY(adxlEv.acceleration.x, adxlEv.acceleration.y, adxlYawOffsetRad, axr, ayr);
  float adxlRoll  = getRoll(axr, ayr, adxlEv.acceleration.z);
  float adxlPitch = getPitch(axr, ayr, adxlEv.acceleration.z);

  // --- Altitude + VSI ---
  if (bmpOK) {
    unsigned long nowA = millis();
    float dtA = (nowA - lastAltMs) / 1000.0f;
    if (dtA > 0.02f) {
      float a = bmp.readAltitude(QNH_hPa);
      if (!isnan(a)) {
        altFiltered_m = 0.90f * altFiltered_m + 0.10f * a;
        float dAlt = altFiltered_m - lastAlt_m;
        vsi_mps = dAlt / dtA;
        vsiFiltered_mps = 0.85f * vsiFiltered_mps + 0.15f * vsi_mps;
        lastAlt_m = altFiltered_m;
      }
      lastAltMs = nowA;
    }
  }

  float rollToSend, pitchToSend;

  // --- FAIL MODE: use ADXL attitude; keep last heading ---
  if (mpuFail) {
    rollToSend  = adxlRoll;
    pitchToSend = adxlPitch;

    drawBankMarkers(adxlRoll);
    drawHorizon(adxlRoll, adxlPitch);
    drawFailMsg();
    drawAltVSI();
    display.display();

    // ✅ CSV ONLY (no labels)
    Serial.print(rollToSend); Serial.print(",");
    Serial.print(pitchToSend); Serial.print(",");
    Serial.print(altFiltered_m); Serial.print(",");
    Serial.print(vsiFiltered_mps); Serial.print(",");
    Serial.println(heading_deg);
    delay(18);
    return;
  }

  // --- NORMAL MODE: MPU + complementary filter + heading integration ---
  sensors_event_t am, gm, tm;
  if (!mpu.getEvent(&am, &gm, &tm)) { mpuFail = true; return; }

  float nowMs = millis();
  float dt = (nowMs - lastTime) / 1000.0f;
  if (dt < 0.0005f) dt = 0.0005f; // avoid div-by-zero
  lastTime = nowMs;

  float mpuRoll  = getRoll (am.acceleration.x, am.acceleration.y, am.acceleration.z);
  float mpuPitch = getPitch(am.acceleration.x, am.acceleration.y, am.acceleration.z);

  float gyroRollRate  = gm.gyro.x * 57.2958f; // deg/s
  float gyroPitchRate = gm.gyro.y * 57.2958f; // deg/s
  float gyroYawRate   = gm.gyro.z * 57.2958f; // deg/s

  filteredRoll  = 0.98f*(filteredRoll  + gyroRollRate  * dt) + 0.02f*mpuRoll;
  filteredPitch = 0.98f*(filteredPitch + gyroPitchRate * dt) + 0.02f*mpuPitch;

  // Integrate heading; wrap 0..360
  heading_deg += gyroYawRate * dt;
  while (heading_deg >= 360.0f) heading_deg -= 360.0f;
  while (heading_deg <    0.0f) heading_deg += 360.0f;

  // Mismatch detection -> failover
  float diffR = fabsf(filteredRoll  - adxlRoll);
  float diffP = fabsf(filteredPitch - adxlPitch);
  if (diffR > 15 || diffP > 15) mismatchCount++; else mismatchCount = 0;
  if (mismatchCount > 12) { mpuFail = true; }

  rollToSend  = filteredRoll;
  pitchToSend = filteredPitch;

  // --- DRAW ---
  drawBankMarkers(filteredRoll);
  drawHorizon(filteredRoll, filteredPitch);
  drawAltVSI();

  display.setCursor(0, 0);
  display.print("R:"); display.print((int)filteredRoll);
  display.print(" P:"); display.print((int)filteredPitch);
  display.display();

  // ✅ CSV ONLY for Processing (roll,pitch,alt,vsi,heading)
  Serial.print(rollToSend);      Serial.print(",");
  Serial.print(pitchToSend);     Serial.print(",");
  Serial.print(altFiltered_m);   Serial.print(",");
  Serial.print(vsiFiltered_mps); Serial.print(",");
  Serial.println(heading_deg);

  delay(18);
}
