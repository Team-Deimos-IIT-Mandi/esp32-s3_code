//FL

/*
 * ESP32-S3 MINI – SINGLE MOTOR PID CONTROLLER (STEALTH SAFETY EDITION)
 * Team Deimos IIT Mandi
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

// ================== RGB LED CONFIG ==================
#define RGB_LED_PIN    6    // CHANGE THIS to your actual LED pin
#define NUM_PIXELS     2
#define LED_BRIGHTNESS 50    // Low brightness to save power

Adafruit_NeoPixel statusLed(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ================== CAN CONFIG ==================
#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2
#define CAN_ID_MOTOR 0x124   // CHANGE PER WHEEL

// ================== MOTOR CONFIG =================
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define PWM_PIN       10
#define DIR_PIN       11
#define PCNT_UNIT     PCNT_UNIT_0
#define ENCODER_PPR   400

// ================== PID CONSTANTS =================
#define KP 0.1369
#define KI 2.4825
#define KD 0.0

#define MOTOR_GAIN   10.4521
#define MOTOR_OFFSET 146.3001


// ================== CONTROL PARAMS =================
#define PWM_FREQ         25000
#define PWM_RESOLUTION   8
#define PWM_MAX          255
#define MIN_PWM_OUTPUT   20

#define RPM_SAMPLE_TIME_MS   50
#define CONTROL_LOOP_TIME_MS 50
#define EMA_ALPHA            0.3
#define RPM_DEADBAND         5.0
#define INTEGRAL_LIMIT       1000.0

#define MAX_CAN_SPEED 1000
#define MAX_RPM       2500
#define CAN_TIMEOUT_MS 500

// ================== CAN STRUCT ==================
typedef struct {
  int16_t speed;     // magnitude
  int8_t  direction; // 1 = forward, 0 = reverse
} __attribute__((packed)) motor_cmd_t;

// ================== VARIABLES ==================
float targetRPM = 0.0;
float currentRPM = 0.0;
float filteredRPM = 0.0;

float errorSum = 0.0;
float lastError = 0.0;
bool firstRPM = true;

int currentPWM = 0;

unsigned long lastRPMTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastCANUpdate = 0;

// Safety Variables
bool encoderFaultDetected = false;
bool canTimeoutDetected = false;
bool isActiveBraking = false;
unsigned long lastBlinkTime = 0;
bool blinkState = false;

// ================== FUNCTION DECL ==================
bool initCAN();
void processCAN();
void checkCANTimeout();
void initPCNT();
void measureRPM();
void runPID();
float calculateRPM(int16_t count);
void checkEncoderHealth();
void updateSafetyLED();

// ================== SETUP ==================
void setup() {
  // 1. SAFETY FIRST: Hard Stop
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); 

  Serial.begin(115200);
  delay(500);

  // 2. Initialize LED
  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  statusLed.show(); // Start OFF

  initCAN();

  // 3. Attach PWM
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);

  initPCNT();

  lastRPMTime = millis();
  lastControlTime = millis();
  lastCANUpdate = millis();

  Serial.println("✓ Single Motor ESP Ready (Stealth Safety)");
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();

  processCAN();
  checkCANTimeout();
  checkEncoderHealth(); // Check for faults
  updateSafetyLED();    // Update visual status

  // FIX 1: Prevent Timing Drift
  if (now - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureRPM();
    lastRPMTime += RPM_SAMPLE_TIME_MS; 
  }

  if (now - lastControlTime >= CONTROL_LOOP_TIME_MS) {
    runPID();
    lastControlTime += CONTROL_LOOP_TIME_MS; 
  }
}

// ================== SAFETY FUNCTIONS ==================

// 1. Detect if Encoder is disconnected or stalled
void checkEncoderHealth() {
  // LOGIC: If PWM is high (>100) BUT RPM is basically zero (<10)
  // It means the motor has power but isn't moving (Disconnected wire or Stall)
  if (abs(currentPWM) > 100 && abs(currentRPM) < 10.0) {
    encoderFaultDetected = true;
  } else {
    encoderFaultDetected = false; // Auto-recover if RPM returns
  }
}

// 2. Manage LED Logic (STEALTH MODE)
void updateSafetyLED() {
  unsigned long now = millis();
  
  // Blink Timer (300ms Toggle)
  if (now - lastBlinkTime > 300) {
    blinkState = !blinkState;
    lastBlinkTime = now;
  }

  uint32_t color = 0; // Default to OFF (Black)

  // PRIORITY 1: CAN TIMEOUT (Blink purple)
  if (canTimeoutDetected) {
    if (blinkState) color = statusLed.Color(255, 0, 255); 
    else color = 0;
  }
  
  // PRIORITY 2: ENCODER FAULT (Blink YELLOW)
  else if (encoderFaultDetected) {
    if (blinkState) color = statusLed.Color(255, 160, 0);
    else color = 0;
  }

  // PRIORITY 3: ACTIVE BRAKING (blink lime) - Warning/Info
  else if (isActiveBraking) {
    if (blinkState) color = statusLed.Color(50, 250, 50);
    else color = 0;
  }
  
  // PRIORITY 4: NORMAL OPERATION (LED OFF)
  else {
    color = 0; // LED stays OFF when everything is working
  }

  statusLed.fill(color);
  statusLed.show();
}

// ================== CAN ==================
bool initCAN() {
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = {
    .acceptance_code = 0,
    .acceptance_mask = 0xFFFFFFFF,
    .single_filter = true
  };

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      twai_start();
      return true;
  }
  return false;
}

void processCAN() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.identifier != CAN_ID_MOTOR) continue;
    if (msg.data_length_code != sizeof(motor_cmd_t)) continue;

    motor_cmd_t cmd;
    memcpy(&cmd, msg.data, sizeof(cmd));

    float rpm = ((float)cmd.speed * MAX_RPM) / MAX_CAN_SPEED;
    if (cmd.direction == 0) rpm = -rpm;

    targetRPM = rpm;
    lastCANUpdate = millis();
  }
}

void checkCANTimeout() {
  if (millis() - lastCANUpdate > CAN_TIMEOUT_MS) {
    targetRPM = 0;
    canTimeoutDetected = true; // Set Flag for LED
  } else {
    canTimeoutDetected = false; // Clear Flag
  }
}

// ================== ENCODER ==================
void initPCNT() {
  pcnt_config_t ch0 = {
    .pulse_gpio_num = ENCODER_A_PIN,
    .ctrl_gpio_num  = ENCODER_B_PIN,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_DEC,
    .neg_mode       = PCNT_COUNT_INC,
    .counter_h_lim  = 32767,
    .counter_l_lim  = -32768,
    .unit           = PCNT_UNIT,
    .channel        = PCNT_CHANNEL_0
  };
  pcnt_unit_config(&ch0);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

void measureRPM() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  pcnt_counter_clear(PCNT_UNIT); 

  float rawRPM = calculateRPM(count);

  if (firstRPM) {
    filteredRPM = rawRPM;
    firstRPM = false;
  } else {
    filteredRPM = EMA_ALPHA * rawRPM + (1.0 - EMA_ALPHA) * filteredRPM;
  }
  currentRPM = filteredRPM;
}

float calculateRPM(int16_t count) {
  const int CPR = ENCODER_PPR * 4; 
  return (float)count * 60000.0 / (CPR * RPM_SAMPLE_TIME_MS);
}

// ================== PID (REFINED) ==================
void runPID() {
  
  /*if (fabs(targetRPM) < 0.1) {
    ledcWrite(PWM_PIN, 0);
    errorSum = 0;   
    lastError = 0;
    currentPWM = 0;
    return;
  }*/

  float error = targetRPM - currentRPM;

  // If we are stopped and the wheel is TRULY still, relax the motor to save heat.
  if (fabs(targetRPM) < 0.1 && fabs(currentRPM) < 5.0 && fabs(errorSum) < 1.0) {
      ledcWrite(PWM_PIN, 0);
      currentPWM = 0;
      errorSum = 0; 
      isActiveBraking = false; // We are relaxed, not fighting
      return; 
  }

  // --- NEW: Detect Active Braking ---
  // If we want to stop (Target ~ 0) but the motor is powered (PWM > 40), we are fighting.
  if (fabs(targetRPM) < 0.1 && abs(currentPWM) > 40) {
      isActiveBraking = true;
  } else {
      isActiveBraking = false;
  }

  if (fabs(error) < RPM_DEADBAND) error = 0;

  float p = KP * error;

  bool saturated = (currentPWM >= PWM_MAX && error > 0) || (currentPWM <= -PWM_MAX && error < 0);
  
  if (!saturated) {
      errorSum += error * (CONTROL_LOOP_TIME_MS / 1000.0);
      errorSum = constrain(errorSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  float i = KI * errorSum;
  float d = KD * (error - lastError);

  float ffMag = (fabs(targetRPM) + MOTOR_OFFSET) / MOTOR_GAIN;
  float ff = (targetRPM >= 0) ? ffMag : -ffMag;

  float control = ff + p + i + d;

  if (control >= 0) {
    digitalWrite(DIR_PIN, HIGH);
    currentPWM = (int)control;
  } else {
    digitalWrite(DIR_PIN, LOW);
    currentPWM = (int)fabs(control);
  }

  if (currentPWM > 0 && currentPWM < MIN_PWM_OUTPUT) {
      currentPWM = MIN_PWM_OUTPUT;
  }

  currentPWM = constrain(currentPWM, 0, PWM_MAX);
  ledcWrite(PWM_PIN, currentPWM);

  lastError = error;
}