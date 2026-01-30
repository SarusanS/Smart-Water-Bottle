#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"
#include "SparkFun_BMI270_Arduino_Library.h" 
#include <Adafruit_AHRS.h>                  

// --- CONSTANTS & CONFIGURATION ---
#define I2C_SDA_PIN 10
#define I2C_SCL_PIN 20
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
const int TOUCH_PIN = 1;  
const int LED_PIN   = 2;  
const int analogPin = 4;

const unsigned long DEBOUNCE_MS = 40;
const unsigned long MULTI_GAP_MS = 300;
const unsigned long DISPLAY_TIMEOUT_MS = 15000;
const unsigned long SETTING_IDLE_TIMEOUT_MS = 3000;
const unsigned long IMU_READ_INTERVAL_MS = 20; 

const int SAMPLE_INTERVAL = 500;   
const int AVG_PERIOD = 10000;      
const int MAX_DEVIATION = 10;      
const float crossArea_dm2 = 0.4225; 
const float TILT_THRESHOLD_DEGREES = 10.0; 
const float STABILITY_CHANGE_THRESHOLD = 10.0; 
#define MAX_SAMPLES (AVG_PERIOD / SAMPLE_INTERVAL)
int samples[MAX_SAMPLES];

const unsigned long STABILITY_WAIT_MS = 15000; 

// --- GLOBAL OBJECTS ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
BMI270 imu;             
Adafruit_Madgwick filter; 

// --- GLOBAL STATE VARIABLES ---
int currentMode = 1; 
float initialdistance = 0;
float lastdistance = 0;
float lastAngleX = 0.0, lastAngleY = 0.0; 
bool prev_unstable = false;
unsigned long lastSensorReadTime = 0, stabilityCheckStartTime = 0; 
bool waitingForStability = false;

float volumeLimitLiters = 0.0, consumedVolume = 0.0; 
int settingValue = 0;
unsigned long lastvolumesettime = 0;
float lastdayremaing = 0, lastdayconsumbtion = 0;
float voltage = 0.0;

// Time tracking
unsigned long previousMillis = 0;
int hour = 0, minute = 0, second = 0;
int month = 1, day = 1, year = 2025;

// Initial Setup
bool isInitialSetup = true; 
int setupStep = 0; 

volatile int touchCount = 0;
unsigned long lastTouchTime = 0, lastDebounceTime = 0;
int lastStableState = LOW;
bool waitingForMore = false;
bool displayAwake = false, inSettingMode = false, alreadyreset = false;

// --- FUNCTION DECLARATIONS ---
void setupI2C(); void initOLED(); void initBMI270();
void updateIMUAngles(float& p, float& r);
bool isTilted(float p, float r); bool isStable(float p, float r);
float getAverageDistanceDM(); void updatevolume();
void handleVolumeSensing(unsigned long now); void handleTouch();
void classifyTouch(int count); void displaySleep();
void displayStatus(); void displayLimitSetting();
void displayModeSwitch(); void displayResetConfirmMode2();
void handleDisplayTimeout(); void dayreset();
void updateBatteryVoltage(); void updateClock();

// --- SETUP ---
void setup() {
  Serial.begin(9600);
  pinMode(TOUCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  setupI2C(); 
  Serial.println("I2C Bus Initialized.");
  initOLED();
  
  while (isInitialSetup) {
    handleTouch(); 
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(" INITIAL TIME SETUP");
    display.drawFastHLine(0, 12, 128, 1);
    display.setCursor(0, 48); display.println("1:Set | 2:Next");
    display.setCursor(0, 56); display.println("3:Back");
    display.setTextSize(2);
    display.setCursor(0, 25);
    if (setupStep == 0)      display.printf("MONTH: %02d", month);
    else if (setupStep == 1) display.printf("DAY:   %02d", day);
    else if (setupStep == 2) display.printf("HOUR:  %02d", hour);
    else if (setupStep == 3) display.printf("MIN:   %02d", minute);
    display.display();
    delay(10); 
  }

  // --- GREETING & INITIALIZATION SCREEN ---
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 20);
  display.println("WELCOME!");
  display.setTextSize(1);
  display.setCursor(15, 45);
  display.println("Initializing...");
  display.display();

  if (!lox.begin()) { Serial.println("❌ Failed to find VL53L0X sensor!"); }
  else { Serial.println("✅ VL53L0X sensor initialized!"); }
  
  initBMI270();
  filter.begin(50); 
  Serial.println("✅ BMI270 Checker Ready.");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  delay(1500); 

  displayAwake = true;
  lastTouchTime = millis(); 

  initialdistance = getAverageDistanceDM();
  lastdistance = initialdistance;
  Serial.printf("Initial distance: %.2f dm\n", initialdistance);
  Serial.println("✅ System Ready (Sleep Mode)");
  previousMillis = millis(); 
}

void loop() {
  unsigned long now = millis();
  handleVolumeSensing(now);
  handleTouch();
  updateClock();
  
  if (displayAwake && !inSettingMode) displayStatus();
  handleDisplayTimeout();
  
  if (currentMode == 1) dayreset(); 
  delay(10); 
}

// --- TOUCH HANDLING ---
void handleTouch() {
  int raw = digitalRead(TOUCH_PIN);
  unsigned long now = millis();
  if (raw != lastStableState && now - lastDebounceTime >= DEBOUNCE_MS) {
    lastStableState = raw; lastDebounceTime = now;
    if (raw == HIGH) { touchCount++; waitingForMore = true; lastTouchTime = now; Serial.printf("Tap #%d detected\n", touchCount); }
  }
  if (waitingForMore && (now - lastTouchTime >= MULTI_GAP_MS)) {
    classifyTouch(touchCount); touchCount = 0; waitingForMore = false;
  }
}

void classifyTouch(int count) {
  if (isInitialSetup) {
    // if (count == 1) { if (++month > 12) month = 1; } 
    // else if (count == 2) { setupStep++; if (setupStep > 3) isInitialSetup = false; }
    // else if (count == 3) { setupStep--; if (setupStep < 0) setupStep = 0; }
    // return;

    if (count == 1) { // Single: Set
      if (setupStep == 0) { month++; if (month > 12) month = 1; }
      else if (setupStep == 1) { day++; if (day > 31) day = 1; }
      else if (setupStep == 2) { hour++; if (hour > 23) hour = 0; }
      else if (setupStep == 3) { minute++; if (minute > 59) minute = 0; }
    } 
    else if (count == 2) { // Double: Next
      setupStep++;
      if (setupStep > 3) isInitialSetup = false;
    }
    else if (count == 3) { // Triple: Back
      setupStep--;
      if (setupStep < 0) setupStep = 0;
    }
    return;
  }
  if (count == 1) { Serial.println("➡ Single Touch"); } 
  else if (count == 2 && !displayAwake) { Serial.println("➡ Double Touch - Wake Display"); displayAwake = true; inSettingMode = false; lastTouchTime = millis(); } 
  else if (count == 2 && displayAwake) { if (currentMode == 1) { Serial.println("➡ Double Touch - Goal Setting"); displayLimitSetting(); } else { Serial.println("➡ Double Touch - Reset Confirm"); displayResetConfirmMode2(); } } 
  else if (count == 3 && displayAwake) { Serial.println("➡ Triple Touch - Switch Mode"); displayModeSwitch(); }
}

// --- MENUS ---
void displayModeSwitch() {
  inSettingMode = true; int target = (currentMode == 1) ? 2 : 1; unsigned long start = millis();
  while (millis() - start < SETTING_IDLE_TIMEOUT_MS) {
    display.clearDisplay(); display.setCursor(0,0); display.println("   SWITCH MODE");
    display.drawFastHLine(0, 10, 128, 1); display.setCursor(0, 25); display.printf("To Mode %d?", target);
    display.setCursor(0, 45); display.println("Double Tap: OK"); display.display();
    int raw = digitalRead(TOUCH_PIN); unsigned long now = millis();
    if (raw == HIGH && now - lastDebounceTime > 150) { 
       static int localTap = 0; localTap++; lastDebounceTime = now;
       if (localTap >= 2) { currentMode = target; consumedVolume = 0; Serial.printf("Mode Changed to %d\n", currentMode); localTap = 0; break; }
       delay(200);
    }
    delay(10);
  }
  inSettingMode = false;
}

void displayResetConfirmMode2() {
  inSettingMode = true; unsigned long start = millis();
  while (millis() - start < SETTING_IDLE_TIMEOUT_MS) {
    display.clearDisplay(); display.setCursor(0, 10); display.println("  RESET TRACKING?");
    display.setCursor(0, 40); display.println("Double Tap: OK"); display.display();
    int raw = digitalRead(TOUCH_PIN); unsigned long now = millis();
    if (raw == HIGH && now - lastDebounceTime > 150) {
       static int localR = 0; localR++; lastDebounceTime = now;
       if (localR >= 2) { consumedVolume = 0; Serial.println("✅ Mode 2 Reset Confirmed"); localR = 0; break; }
       delay(200);
    }
  }
  inSettingMode = false;
}

void displayLimitSetting() {
  unsigned long lastInputTime = millis(); bool confirmed = false; int settingTouchCount = 0; unsigned long touchStart = 0; const unsigned long doubleTouchGap = 200; inSettingMode = true;
  while (inSettingMode) { 
    unsigned long now = millis(); int raw = digitalRead(TOUCH_PIN); static int lastState = LOW; static unsigned long lastDebounce = 0;
    if (raw != lastState && now - lastDebounce > 50) { lastState = raw; lastDebounce = now; if (raw == HIGH) { settingTouchCount++; touchStart = now; lastInputTime = now; } }
    if (settingTouchCount == 2 && (now - touchStart < doubleTouchGap)) { volumeLimitLiters = (float)settingValue; consumedVolume = 0; lastvolumesettime = millis(); confirmed = true; alreadyreset = false; break; }
    if (settingTouchCount == 1 && (now - touchStart > doubleTouchGap)) { settingValue++; if (settingValue > 15) settingValue = 0; settingTouchCount = 0; }
    if (now - lastInputTime > SETTING_IDLE_TIMEOUT_MS) { confirmed = false; break; }
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0); display.println("   SET DAILY GOAL (L) "); display.drawFastHLine(0, 10, 128, 1);
    display.setCursor(0, 14); display.println("1 Tap: Increment\n2 Taps: Confirm\nIdle 3s: Cancel");
    display.setTextSize(4); display.setCursor(30, 38); display.printf("%2d", settingValue); display.display(); delay(10);
  }
  display.clearDisplay(); display.setTextSize(2); display.setCursor(10, 20);
  if (confirmed) { display.printf("Saved: %dL", settingValue); Serial.printf("Goal Set to: %d L\n", settingValue); } else { display.println("Cancelled"); }
  display.display(); delay(1200); inSettingMode = false; displayAwake = true; lastTouchTime = millis();
}

// --- STATUS DISPLAY ---
void displayStatus() {
  updateBatteryVoltage();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 0); display.printf("%02d:%02d", hour, minute); 
  display.setCursor(85, 0); display.printf("%02d/%02d", day, month); 
  int x = 54; display.drawRect(x, 0, 20, 10, 1); display.fillRect(x + 20, 3, 2, 4, 1); 
  if (voltage > 3.7) { display.fillRect(x+2, 2, 4, 6, 1); display.fillRect(x+8, 2, 4, 6, 1); display.fillRect(x+14, 2, 4, 6, 1); }
  else if (voltage > 3.6) { display.fillRect(x+2, 2, 4, 6, 1); display.fillRect(x+8, 2, 4, 6, 1); }
  else if (voltage >= 3.5) { display.fillRect(x+2, 2, 4, 6, 1); }
  display.drawFastHLine(0, 12, 128, 1);
  display.setTextSize(2);
  display.setCursor(0, 18);
  if (currentMode == 1) {
    display.printf("Goal:%.1f", volumeLimitLiters);
    if (consumedVolume >= volumeLimitLiters) { display.drawLine(110, 24, 113, 28, 1); display.drawLine(113, 28, 119, 18, 1); }
    display.setCursor(0, 34); display.printf("Used:%.1f", consumedVolume);
    display.setTextSize(1); display.setCursor(0, 52); display.printf("yes_con: %.1f L", lastdayconsumbtion);
  } else {
    display.println("Continuous");
    display.setCursor(0, 34); display.printf("Used:%.1f", consumedVolume);
    display.setTextSize(1); display.setCursor(0, 52); display.println("Mode 2: Tracking Mode");
  }
  display.display();
}

// --- CORE UTILS ---
void updateClock() {
  unsigned long cur = millis();
  while (cur - previousMillis >= 1000) { previousMillis += 1000; second++;
    if (second >= 60) { second = 0; minute++; if (minute >= 60) { minute = 0; hour++; if (hour >= 24) { hour = 0; day++; } } }
  }
}

void dayreset() {
  if (millis() - lastvolumesettime >= 86400000 && !alreadyreset) { 
    lastdayconsumbtion = consumedVolume; consumedVolume = 0; alreadyreset = true;
    Serial.println("✅ Daily consumption reset.");
  }
}

void handleVolumeSensing(unsigned long now) {
  float currentPitch, currentRoll;
  if (now - lastSensorReadTime >= IMU_READ_INTERVAL_MS) {
    updateIMUAngles(currentPitch, currentRoll);
    lastSensorReadTime = now;
    bool tilted = isTilted(currentPitch, currentRoll);
    bool stable = isStable(currentPitch, currentRoll);
    Serial.printf("Roll(X): %.2f deg | Pitch(Y): %.2f deg | Tilted: %s | Stable: %s\n",
                  currentRoll, currentPitch, tilted ? "TRUE" : "FALSE", stable ? "TRUE" : "FALSE");
    if (stable && !tilted) { lastAngleX = currentPitch; lastAngleY = currentRoll; }
    if (!stable || tilted) prev_unstable = true; 
    if (stable && !tilted && prev_unstable && !waitingForStability) {
      Serial.println("🔴🔴🔴Motion stopped. Starting stability verification🔴🔴🔴...");
      stabilityCheckStartTime = now; waitingForStability = true;
    }
  }
  if (waitingForStability && (now - stabilityCheckStartTime >= STABILITY_WAIT_MS)) {
    Serial.println("🔴Wait period elapsed. Verifying stability again...");
    float vPitch, vRoll; updateIMUAngles(vPitch, vRoll);
    if (isStable(vPitch, vRoll) && !isTilted(vPitch, vRoll)) {
        Serial.println("✅ Stability confirmed. Updating volume calculation");
        updatevolume(); prev_unstable = false; 
    } else { Serial.println("❌❌❌ Still tilted or unstable, skipping update."); }
    waitingForStability = false; 
  }
}

void updatevolume() {
  float current_distance = getAverageDistanceDM();
  float delta_distance = current_distance - lastdistance;
  float delta_volume = delta_distance * crossArea_dm2;
  Serial.printf("Current: %.2f dm | Last: %.2f dm | Δ: %.3f dm\n", current_distance, lastdistance, delta_distance);
  if (delta_distance > 0.05) { consumedVolume += delta_volume; Serial.printf("💧 Water consumed. Volume +%.3f dm³ | Total consumed: %.3f dm³\n", delta_volume, consumedVolume); } 
  else if (delta_distance < -0.2) { Serial.println("🔄 Bottle refilled."); } 
  lastdistance = current_distance;
}

float getAverageDistanceDM() {
  unsigned long startTime = millis(); int sampleCount = 0;
  while (millis() - startTime < AVG_PERIOD) {
    updateClock(); VL53L0X_RangingMeasurementData_t measure; lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && sampleCount < MAX_SAMPLES) { samples[sampleCount++] = measure.RangeMilliMeter; }
    unsigned long waitStart = millis(); while(millis() - waitStart < SAMPLE_INTERVAL) { updateClock(); delay(5); }
  }
  if (sampleCount == 0) return lastdistance;
  long sum = 0; for (int i = 0; i < sampleCount; i++) sum += samples[i];
  float mean = (float)sum / sampleCount;
  long filteredSum = 0; int filteredCount = 0;
  for (int i = 0; i < sampleCount; i++) { if (abs(samples[i] - mean) <= MAX_DEVIATION) { filteredSum += samples[i]; filteredCount++; } }
  float avg_mm = (filteredCount > 0) ? (float)filteredSum / filteredCount : mean;
  Serial.printf("✅ Averaged Distance: %.1f mm\n", avg_mm);
  return avg_mm / 100.0;
}

void setupI2C() { Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); }
void initOLED() { if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1); display.clearDisplay(); display.display(); }
void initBMI270() { if (imu.beginI2C(BMI2_I2C_PRIM_ADDR) != BMI2_OK) { Serial.println("ERROR: BMI270 not found."); while(1); } imu.performComponentRetrim(); imu.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Z); imu.performGyroOffsetCalibration(); }
void updateIMUAngles(float& p, float& r) { imu.getSensorData(); filter.updateIMU(imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ, imu.data.accelX, imu.data.accelY, imu.data.accelZ); p = filter.getPitch(); r = filter.getRoll(); }
bool isTilted(float p, float r) { return (abs(p) > TILT_THRESHOLD_DEGREES || abs(r) > TILT_THRESHOLD_DEGREES); }
bool isStable(float p, float r) { float dp = abs(p - lastAngleX), dr = abs(r - lastAngleY); return (dp < STABILITY_CHANGE_THRESHOLD && dr < STABILITY_CHANGE_THRESHOLD && abs(sqrt(imu.data.accelX*imu.data.accelX+imu.data.accelY*imu.data.accelY+imu.data.accelZ*imu.data.accelZ)-1.0)<0.1); }
void updateBatteryVoltage() { voltage = (analogRead(analogPin) / 4095.0) * 2.89 * 1.33; }
void handleDisplayTimeout() { if (displayAwake && !inSettingMode && (millis() - lastTouchTime > DISPLAY_TIMEOUT_MS)) { display.clearDisplay(); display.display(); displayAwake = false; } }
void displaySleep() { display.clearDisplay(); display.display(); }