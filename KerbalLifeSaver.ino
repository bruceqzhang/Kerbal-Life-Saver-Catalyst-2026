#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_BMP280 bmp;
MAX30105 particleSensor;

// RGB LED pins (common anode — LOW = on)
const int redPin   = 5;
const int greenPin = 17;
const int bluePin  = 16;

// ──────────────────────────────────────────────
// Safety boundaries
// ──────────────────────────────────────────────
// Heart rate (BPM)
const int HR_LOW_WARN  = 50;
const int HR_LOW_CRIT  = 40;
const int HR_HIGH_WARN = 110;
const int HR_HIGH_CRIT = 150;

// SpO2 (%)
const int SPO2_LOW_WARN = 95;
const int SPO2_LOW_CRIT = 90;

// Temperature (°C)
const int TEMP_LOW_WARN  = 18;
const int TEMP_LOW_CRIT  = 10;
const int TEMP_HIGH_WARN = 35;
const int TEMP_HIGH_CRIT = 40;

// Pressure (kPa)
const float PRES_LOW_WARN  = 98.0;
const float PRES_LOW_CRIT  = 96.0;
const float PRES_HIGH_WARN = 103.0;
const float PRES_HIGH_CRIT = 105.0;

// ──────────────────────────────────────────────
// Sensor state
// ──────────────────────────────────────────────
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength = 100;
int32_t spo2;
int8_t  validSPO2;
int32_t heartRate;
int8_t  validHeartRate;

int currentHeartRate   = -999;
int currentBloodOxygen = -999;
int sampleCounter      = 0;
bool initialBufferFull = false;

unsigned long lastPrintTime   = 0;
unsigned long lastDisplayTime = 0;
const unsigned long PRINT_INTERVAL   = 1000;
const unsigned long DISPLAY_INTERVAL = 200;

int animFrame = 0;

// ──────────────────────────────────────────────
// Status levels
// ──────────────────────────────────────────────
enum Status { GOOD, WARN, CRIT, NO_DATA };

Status getHRStatus(int val) {
  if (val == -999) return NO_DATA;
  if (val < HR_LOW_CRIT  || val > HR_HIGH_CRIT)  return CRIT;
  if (val < HR_LOW_WARN  || val > HR_HIGH_WARN)   return WARN;
  return GOOD;
}

Status getSPO2Status(int val) {
  if (val == -999) return NO_DATA;
  if (val < SPO2_LOW_CRIT) return CRIT;
  if (val < SPO2_LOW_WARN) return WARN;
  return GOOD;
}

Status getTempStatus(int val) {
  if (val < TEMP_LOW_CRIT  || val > TEMP_HIGH_CRIT)  return CRIT;
  if (val < TEMP_LOW_WARN  || val > TEMP_HIGH_WARN)   return WARN;
  return GOOD;
}

Status getPresStatus(float val) {
  if (val < PRES_LOW_CRIT  || val > PRES_HIGH_CRIT)  return CRIT;
  if (val < PRES_LOW_WARN  || val > PRES_HIGH_WARN)   return WARN;
  return GOOD;
}

Status overallStatus(Status a, Status b, Status c, Status d) {
  Status worst = GOOD;
  Status all[] = {a, b, c, d};
  for (int i = 0; i < 4; i++) {
    if (all[i] == CRIT) return CRIT;
    if (all[i] == WARN || all[i] == NO_DATA) worst = WARN;
  }
  return worst;
}

// ──────────────────────────────────────────────
// RGB LED (common anode: LOW = on, HIGH = off)
// ──────────────────────────────────────────────
void setLED(Status s) {
  digitalWrite(redPin,   HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  HIGH);
  switch (s) {
    case GOOD:    digitalWrite(greenPin, LOW); break;
    case WARN:    digitalWrite(redPin, LOW); digitalWrite(greenPin, LOW); break; // red+green = yellow
    case CRIT:    digitalWrite(redPin,   LOW); break;
    case NO_DATA: digitalWrite(bluePin,  LOW); break;
  }
}

// ──────────────────────────────────────────────
// Animated icons (8×7 bitmaps, 2 frames each)
// ──────────────────────────────────────────────
const uint8_t heartIcon[2][7] = {
  { 0b01100110, 0b11111110, 0b11111110, 0b11111110, 0b01111100, 0b00111000, 0b00010000 },
  { 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 }
};

const uint8_t dropIcon[2][7] = {
  { 0b00010000, 0b00111000, 0b01111100, 0b11111110, 0b11111110, 0b11111110, 0b01111100 },
  { 0b00010000, 0b00111000, 0b01101100, 0b11010110, 0b11111110, 0b11111110, 0b01111100 }
};

const uint8_t thermIcon[2][7] = {
  { 0b00001000, 0b00001010, 0b00001000, 0b00001010, 0b00011100, 0b00011100, 0b00001000 },
  { 0b00001000, 0b00001001, 0b00001000, 0b00001001, 0b00011100, 0b00011100, 0b00001000 }
};

const uint8_t waveIcon[2][7] = {
  { 0b00000000, 0b01000100, 0b10101010, 0b00010001, 0b00000000, 0b01000100, 0b10101010 },
  { 0b00000000, 0b00100010, 0b01010101, 0b10001000, 0b00000000, 0b00100010, 0b01010101 }
};

void drawIcon(int x, int y, const uint8_t icon[][7], int frame, uint16_t colour) {
  for (int row = 0; row < 7; row++) {
    for (int col = 0; col < 8; col++) {
      if (icon[frame][row] & (0x80 >> col)) {
        display.drawPixel(x + col, y + row, colour);
      }
    }
  }
}

bool shouldShowValue(Status s, unsigned long now) {
  if (s == GOOD || s == NO_DATA) return true;
  if (s == WARN) return (now / 600) % 2 == 0;
  if (s == CRIT) return (now / 200) % 2 == 0;
  return true;
}

// ──────────────────────────────────────────────
// Setup
// ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);
  digitalWrite(redPin,   HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  LOW); // blue = initialising

  Wire.begin(21, 22);
  bmp.begin(0x76);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 12);
  display.println("  Initialising...");
  display.display();

  particleSensor.begin(Wire, I2C_SPEED_FAST);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode       = 2;
  byte sampleRate    = 100;
  int  pulseWidth    = 411;
  int  adcRange      = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.clearFIFO();
}

// ──────────────────────────────────────────────
// Loop
// ──────────────────────────────────────────────
void loop() {
  particleSensor.check();

  if (particleSensor.getIR() < 40000) {
    currentHeartRate   = -999;
    currentBloodOxygen = -999;
    sampleCounter      = 0;
    initialBufferFull  = false;
    particleSensor.clearFIFO();
  } else {
    while (particleSensor.available()) {
      if (initialBufferFull) {
        memmove(redBuffer, redBuffer + 1, 99 * sizeof(uint32_t));
        memmove(irBuffer,  irBuffer  + 1, 99 * sizeof(uint32_t));
        redBuffer[99] = particleSensor.getFIFORed();
        irBuffer[99]  = particleSensor.getFIFOIR();
        particleSensor.nextSample();
        sampleCounter++;

        if (sampleCounter >= 25) {
          sampleCounter = 0;
          maxim_heart_rate_and_oxygen_saturation(
            irBuffer, bufferLength, redBuffer,
            &spo2, &validSPO2, &heartRate, &validHeartRate);
          if (validHeartRate && heartRate > 40 && heartRate < 200) currentHeartRate   = heartRate;
          if (validSPO2      && spo2 > 70      && spo2 <= 100)     currentBloodOxygen = spo2;
        }
      } else {
        redBuffer[sampleCounter] = particleSensor.getFIFORed();
        irBuffer[sampleCounter]  = particleSensor.getFIFOIR();
        particleSensor.nextSample();
        sampleCounter++;

        if (sampleCounter >= 100) {
          maxim_heart_rate_and_oxygen_saturation(
            irBuffer, bufferLength, redBuffer,
            &spo2, &validSPO2, &heartRate, &validHeartRate);
          if (validHeartRate && heartRate > 40 && heartRate < 200) currentHeartRate   = heartRate;
          if (validSPO2      && spo2 > 70      && spo2 <= 100)     currentBloodOxygen = spo2;
          sampleCounter     = 0;
          initialBufferFull = true;
        }
      }
    }
  }

  // ── Serial print ──
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = millis();
    int temperature  = bmp.readTemperature();
    float pressure   = bmp.readPressure() / 1000.0; // Pa → kPa
    Serial.println("Temperature: " + String(temperature) + " C");
    Serial.println("Pressure: "    + String(pressure, 2) + " kPa");
    Serial.println("Heart Rate: "  + String(currentHeartRate) + " BPM");
    Serial.println("Blood Oxygen: "+ String(currentBloodOxygen) + " %");
    Serial.println("---");
  }

  // ── Display + LED update ──
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = millis();
    animFrame = (animFrame + 1) % 2;

    int temperature  = bmp.readTemperature();
    float pressure   = bmp.readPressure() / 1000.0; // Pa → kPa

    Status hrStatus   = getHRStatus(currentHeartRate);
    Status spo2Status = getSPO2Status(currentBloodOxygen);
    Status tempStatus = getTempStatus(temperature);
    Status presStatus = getPresStatus(pressure);
    Status overall    = overallStatus(hrStatus, spo2Status, tempStatus, presStatus);

    setLED(overall);

    unsigned long now = millis();
    display.clearDisplay();

    // Row 0 — Heart Rate  (y=0)
    drawIcon(0, 0, heartIcon, animFrame, WHITE);
    if (shouldShowValue(hrStatus, now)) {
      display.setCursor(11, 0);
      display.print("BPM: ");
      if (currentHeartRate == -999) display.print("--");
      else display.print(currentHeartRate);
    }

    // Row 1 — SpO2  (y=8)
    drawIcon(0, 8, dropIcon, animFrame, WHITE);
    if (shouldShowValue(spo2Status, now)) {
      display.setCursor(11, 8);
      display.print("SpO2: ");
      if (currentBloodOxygen == -999) display.print("--");
      else { display.print(currentBloodOxygen); display.print("%"); }
    }

    // Row 2 — Temperature  (y=16)
    drawIcon(0, 16, thermIcon, animFrame, WHITE);
    if (shouldShowValue(tempStatus, now)) {
      display.setCursor(11, 16);
      display.print("Temp: ");
      display.print(temperature);
      display.print("C");
    }

    // Row 3 — Pressure  (y=24)
    drawIcon(0, 24, waveIcon, animFrame, WHITE);
    if (shouldShowValue(presStatus, now)) {
      display.setCursor(11, 24);
      display.print("Pres: ");
      display.print(pressure, 1);
      display.print("kPa");
    }

    display.display();
  }
}