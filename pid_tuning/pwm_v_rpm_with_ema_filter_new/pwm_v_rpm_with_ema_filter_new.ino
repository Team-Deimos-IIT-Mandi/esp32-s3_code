/*
 * ESP32-S3 PWM Control with Quadrature Encoder RPM Measurement (x4 Mode)
 * CSV Output Format: PWM, Raw Tyre RPM, Filtered Tyre RPM
 * Feature: Smooth Ramp Up and Smooth Ramp Down
 * * Hardware Connections:
 * - Encoder Channel A: GPIO 4
 * - Encoder Channel B: GPIO 5
 * - PWM Output: GPIO 10
 * - Direction Output: GPIO 11
 */

#include "driver/pcnt.h"

// Pin Definitions
#define ENCODER_A_PIN 5
#define ENCODER_B_PIN 4
#define PWM_PIN 10
#define DIR_PIN 11

// Encoder and Gearbox Specifications
#define ENCODER_PPR 400        // Pulses Per Revolution
#define GEAR_RATIO 13.7        // Motor to Tyre Gear Ratio
#define PCNT_UNIT PCNT_UNIT_0  // PCNT unit to use

// PWM Configuration
#define PWM_FREQ 20000         // 25 kHz PWM frequency
#define PWM_RESOLUTION 8       // 8-bit resolution (0-255)

// Timing Parameters
#define RPM_SAMPLE_TIME_MS 50      // Sample time for RPM calculation (50ms)
#define PWM_INCREMENT_TIME_MS 100   // Time between PWM steps (100ms)
#define WAIT_BEFORE_START_MS 2000   // Wait 2 seconds before starting
#define WAIT_AT_MAX_PWM_MS 5000     // Stay at PWM 255 for 5 seconds

// EMA Filter Configuration
#define EMA_ALPHA 0.3              // EMA smoothing factor

// Global Variables
unsigned long lastRPMTime = 0;
unsigned long lastPWMTime = 0;
unsigned long pwmStartTime = 0;
unsigned long maxPWMStartTime = 0;
int currentPWM = 0;
int lastPrintedPWM = -1;  // Track last PWM value that was printed

// State Machine Flags
bool waitingToStart = true;
bool rampingUp = true;      // Start by ramping up
bool atMaxPWM = false;
bool rampingDown = false;   // New flag for ramp down

// Variable to store previous counter value for delta calculation
int16_t lastPulseCount = 0; 

// EMA Filter Variables
float filteredMotorRPM = 0.0;
bool firstRPMReading = true;
bool headerPrinted = false;

void setup() {
  Serial.begin(115200);
  
  delay(1000);
  
  // Initialize Direction Pin
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);  // Set direction (HIGH = forward)
  
  // Configure PWM
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);  // Start with PWM = 0
  
  // Initialize PCNT for Quadrature Encoder (x4 mode)
  initPCNT();
  
  lastRPMTime = millis();
  lastPWMTime = millis();
  pwmStartTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // 1. Initial Waiting Period
  if (waitingToStart) {
    if (currentTime - pwmStartTime >= WAIT_BEFORE_START_MS) {
      waitingToStart = false;
      rampingUp = true;
      
      if (!headerPrinted) {
        // UPDATED HEADER: Tyre RPM only
        Serial.println("PWM,Raw Tyre RPM,Filtered Tyre RPM");
        headerPrinted = true;
      }
      
      // Reset logic
      pcnt_counter_clear(PCNT_UNIT);
      lastPulseCount = 0;
      lastRPMTime = currentTime;
      lastPWMTime = currentTime;
    }
    return;
  }
  
  // 2. Measure RPM (Always runs)
  if (currentTime - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureAndDisplayRPM();
    lastRPMTime = currentTime;
  }
  
  // 3. PWM Logic State Machine
  if (currentTime - lastPWMTime >= PWM_INCREMENT_TIME_MS) {
    
    // State A: Ramping UP
    if (rampingUp && !atMaxPWM && !rampingDown) {
      currentPWM++;
      if (currentPWM >= 255) {
        currentPWM = 255;
        atMaxPWM = true;
        rampingUp = false;
        maxPWMStartTime = currentTime;
      }
      ledcWrite(PWM_PIN, currentPWM);
      lastPWMTime = currentTime;
    }
    
    // State B: Holding at Max Speed
    else if (atMaxPWM) {
      if (currentTime - maxPWMStartTime >= WAIT_AT_MAX_PWM_MS) {
        atMaxPWM = false;
        rampingDown = true; // Start ramping down instead of resetting
      }
      // Update timer so we don't skip checks
      lastPWMTime = currentTime; 
    }
    
    // State C: Ramping DOWN (New Feature)
    else if (rampingDown) {
      currentPWM--;
      if (currentPWM <= 0) {
        currentPWM = 0;
        rampingDown = false;
        
        // Cycle Complete: Reset to start over
        waitingToStart = true;
        pwmStartTime = currentTime;
        
        // Reset Filter logic
        firstRPMReading = true;
        filteredMotorRPM = 0.0;
        lastPrintedPWM = -1;
        
        pcnt_counter_clear(PCNT_UNIT);
        lastPulseCount = 0;
      }
      ledcWrite(PWM_PIN, currentPWM);
      lastPWMTime = currentTime;
    }
  }
}

void measureAndDisplayRPM() {
  // 1. Read the counter
  int16_t currentCount;
  pcnt_get_counter_value(PCNT_UNIT, &currentCount);
  
  // 2. Clear the counter immediately so it doesn't overflow
  pcnt_counter_clear(PCNT_UNIT);
  
  // 3. Reset our software tracker (since we just cleared hardware)
  lastPulseCount = 0; 
  
  // 4. Calculate RPM using the count we just read (which represents the delta)
  // Note: currentCount IS the delta because we started from 0
  float rawMotorRPM = calculateRPM(currentCount, RPM_SAMPLE_TIME_MS);
  
  // 5. Apply Filter
  filteredMotorRPM = applyEMAFilter(rawMotorRPM);
  
  // 6. Calculate Tyre RPM
  float rawTyreRPM = rawMotorRPM / GEAR_RATIO;
  float filteredTyreRPM = filteredMotorRPM / GEAR_RATIO;
  
  // Print
  if (currentPWM != lastPrintedPWM || currentPWM == 0 || currentPWM == 255) {
    Serial.print(currentPWM);
    Serial.print(",");
    Serial.print(rawTyreRPM, 2);
    Serial.print(",");
    Serial.println(filteredTyreRPM, 2);
    
    lastPrintedPWM = currentPWM;
  }
}

float applyEMAFilter(float newValue) {
  if (firstRPMReading) {
    filteredMotorRPM = newValue;
    firstRPMReading = false;
    return filteredMotorRPM;
  }
  return (EMA_ALPHA * newValue) + ((1.0 - EMA_ALPHA) * filteredMotorRPM);
}

void initPCNT() {
  pcnt_config_t pcnt_config_ch0 = {
    .pulse_gpio_num = ENCODER_A_PIN,
    .ctrl_gpio_num = ENCODER_B_PIN,
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_DEC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT,
    .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_ch0);
  
  pcnt_config_t pcnt_config_ch1 = {
    .pulse_gpio_num = ENCODER_B_PIN,
    .ctrl_gpio_num = ENCODER_A_PIN,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_REVERSE,
    .pos_mode = PCNT_COUNT_DEC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT,
    .channel = PCNT_CHANNEL_1,
  };
  pcnt_unit_config(&pcnt_config_ch1);
  
  pcnt_set_filter_value(PCNT_UNIT, 100);
  pcnt_filter_enable(PCNT_UNIT);
  
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

float calculateRPM(int16_t pulseCount, unsigned long elapsedTimeMs) {
  const int countsPerRevolution = ENCODER_PPR * 4;
  return (float)(pulseCount * 60000.0) / (countsPerRevolution * elapsedTimeMs);
}