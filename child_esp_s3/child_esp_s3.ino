/*
 * ESP32-S3 SLAVE MOTOR CONTROLLER - SMOOTH MOTOR RPM WITH REFACTORED PID
 * Change ONLY: CAN_ID_MOTOR, CAN_ID_FEEDBACK, ENCODER_PPR, GEAR_RATIO, and PID values
 * 
 * FL: CAN_ID_MOTOR=0x124, CAN_ID_FEEDBACK=0x224, ENCODER_PPR=400, GEAR_RATIO=13.7
 * FR: CAN_ID_MOTOR=0x121, CAN_ID_FEEDBACK=0x221, ENCODER_PPR=400, GEAR_RATIO=13.7
 * BR: CAN_ID_MOTOR=0x122, CAN_ID_FEEDBACK=0x222, ENCODER_PPR=400, GEAR_RATIO=13.7
 * BL: CAN_ID_MOTOR=0x123, CAN_ID_FEEDBACK=0x223, ENCODER_PPR=600, GEAR_RATIO=13.7
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

// ================== CHANGE FOR EACH WHEEL ==================
#define CAN_ID_MOTOR     0x122   // Command ID
#define CAN_ID_FEEDBACK  0x222   // Feedback ID
#define ENCODER_PPR      400     // Encoder pulses per revolution
#define GEAR_RATIO       13.7    // Motor to Tyre gear ratio

// PID Constants (FR values shown - CHANGE FOR EACH WHEEL)
#define KP 0.3
#define KI 1.0
#define KD 0.01
#define MOTOR_GAIN   0.7632
#define MOTOR_OFFSET 13.41

// Motion Profile (Smoothness)
#define ACCELERATION_STEP 140.0  // Motor RPM change per control loop (smooth ~1s to full speed)
// ===========================================================

// ================== RGB LED CONFIG ==================
#define RGB_LED_PIN    6
#define NUM_PIXELS     2
#define LED_BRIGHTNESS 50

Adafruit_NeoPixel statusLed(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ================== CAN CONFIG ==================
#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2

// ================== MOTOR CONFIG ==================
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define PWM_PIN       10
#define DIR_PIN       11
#define PCNT_UNIT     PCNT_UNIT_0

// ================== CONTROL PARAMS ==================
#define PWM_FREQ         25000
#define PWM_RESOLUTION   8
#define PWM_MAX          255
#define PWM_MIN          0
#define MIN_PWM_OUTPUT   20

#define RPM_SAMPLE_TIME_MS   50
#define CONTROL_LOOP_TIME_MS 50
#define EMA_ALPHA            0.3
#define RPM_DEADBAND         5.0   // Keep original deadband (in motor RPM)
#define INTEGRAL_LIMIT       1000.0

#define MAX_RPM       182.48  // Max tyre RPM (2500 motor RPM / 13.7)
#define CAN_TIMEOUT_MS 500

// ================== CAN STRUCTS ==================
typedef struct {
  float target_rpm;  // Received in TYRE RPM
} __attribute__((packed)) motor_cmd_t;

typedef struct {
  float position;
  float velocity;
} __attribute__((packed)) motor_feedback_t;

// ================== VARIABLES ==================
float finalTargetMotorRPM = 0.0;   // Final destination (converted from CAN)
float activeTargetMotorRPM = 0.0;  // Moving target (ramped)
float currentMotorRPM = 0.0;       // Current motor speed
float filteredMotorRPM = 0.0;

float errorSum = 0.0;
float lastError = 0.0;
bool firstRPM = true;

int currentPWM = 0;
volatile long globalEncoderTicks = 0;

const float TICKS_TO_RAD = (2.0 * PI) / (ENCODER_PPR * 4.0);
const float RPM_TO_RADS = (2.0 * PI) / 60.0;

unsigned long lastRPMTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastCANUpdate = 0;
unsigned long lastFeedbackTime = 0;

bool canTimeoutDetected = false;

// ================== FUNCTION DECLARATIONS ==================
bool initCAN();
void processCAN();
void sendFeedback();
void checkCANTimeout();
void initPCNT();
void measureRPM();
void updateSetpoint();
void runPID();
float calculateRPM(int16_t count);
void updateLED();

// ================== SETUP ==================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);

  Serial.begin(115200);
  delay(500);

  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  statusLed.fill(statusLed.Color(255, 255, 0)); // Yellow = init
  statusLed.show();

  if (!initCAN()) {
    statusLed.fill(statusLed.Color(255, 0, 0)); // Red = CAN failed
    statusLed.show();
    while(1) delay(100);
  }

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);

  initPCNT();

  lastRPMTime = millis();
  lastControlTime = millis();
  lastCANUpdate = millis();
  lastFeedbackTime = millis();

  statusLed.fill(statusLed.Color(0, 255, 0)); // Green = ready
  statusLed.show();

  Serial.println("✓ Slave Ready - Smooth Motor RPM Control (Refactored)");
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();

  processCAN();
  checkCANTimeout();

  if (now - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureRPM();
    lastRPMTime += RPM_SAMPLE_TIME_MS;
  }

  if (now - lastControlTime >= CONTROL_LOOP_TIME_MS) {
    updateSetpoint();  // Ramp the target
    runPID();
    lastControlTime += CONTROL_LOOP_TIME_MS;
  }

  if (now - lastFeedbackTime >= 20) {
    sendFeedback();
    lastFeedbackTime = now;
  }

  updateLED();
}

// ================== CAN ==================
bool initCAN() {
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
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

    // Receive TYRE RPM and immediately convert to Motor RPM
    finalTargetMotorRPM = cmd.target_rpm * GEAR_RATIO;
    lastCANUpdate = millis();
  }
}

void sendFeedback() {
  motor_feedback_t fb;
  fb.position = globalEncoderTicks * TICKS_TO_RAD;
  fb.velocity = (currentMotorRPM / GEAR_RATIO) * RPM_TO_RADS;  // Send tyre velocity

  twai_message_t msg = {};
  msg.identifier = CAN_ID_FEEDBACK;
  msg.data_length_code = sizeof(motor_feedback_t);
  memcpy(msg.data, &fb, sizeof(fb));

  twai_transmit(&msg, 0);
}

void checkCANTimeout() {
  if (millis() - lastCANUpdate > CAN_TIMEOUT_MS) {
    finalTargetMotorRPM = 0;
    canTimeoutDetected = true;
  } else {
    canTimeoutDetected = false;
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
  
  pcnt_config_t ch1 = {
    .pulse_gpio_num = ENCODER_B_PIN,
    .ctrl_gpio_num  = ENCODER_A_PIN,
    .lctrl_mode     = PCNT_MODE_KEEP,
    .hctrl_mode     = PCNT_MODE_REVERSE,
    .pos_mode       = PCNT_COUNT_DEC,
    .neg_mode       = PCNT_COUNT_INC,
    .counter_h_lim  = 32767,
    .counter_l_lim  = -32768,
    .unit           = PCNT_UNIT,
    .channel        = PCNT_CHANNEL_1
  };
  pcnt_unit_config(&ch1);
  
  pcnt_set_filter_value(PCNT_UNIT, 100);
  pcnt_filter_enable(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

void measureRPM() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  globalEncoderTicks += count;
  pcnt_counter_clear(PCNT_UNIT);

  float rawRPM = calculateRPM(count);

  if (firstRPM) {
    filteredMotorRPM = rawRPM;
    firstRPM = false;
  } else {
    filteredMotorRPM = EMA_ALPHA * rawRPM + (1.0 - EMA_ALPHA) * filteredMotorRPM;
  }
  currentMotorRPM = filteredMotorRPM;
}

float calculateRPM(int16_t count) {
  const int CPR = ENCODER_PPR * 4;
  return (float)count * 60000.0 / (CPR * RPM_SAMPLE_TIME_MS);
}

// ================== MOTION PROFILING ==================
void updateSetpoint() {
  // Ramp the active target towards final target (now in Motor RPM)
  if (activeTargetMotorRPM < finalTargetMotorRPM) {
    activeTargetMotorRPM += ACCELERATION_STEP;
    if (activeTargetMotorRPM > finalTargetMotorRPM) {
      activeTargetMotorRPM = finalTargetMotorRPM;
    }
  } else if (activeTargetMotorRPM > finalTargetMotorRPM) {
    activeTargetMotorRPM -= ACCELERATION_STEP;
    if (activeTargetMotorRPM < finalTargetMotorRPM) {
      activeTargetMotorRPM = finalTargetMotorRPM;
    }
  }
}

// ================== PID ==================
void runPID() {
  // Calculate error directly in Motor RPM
  float error = activeTargetMotorRPM - currentMotorRPM;

  // Stop condition - RESET integral when stopped
  if (fabs(activeTargetMotorRPM) < 0.1 && fabs(currentMotorRPM) < 5.0) {
    ledcWrite(PWM_PIN, 0);
    currentPWM = 0;
    errorSum = 0;
    lastError = 0;
    return;
  }

  if (fabs(error) < RPM_DEADBAND) error = 0;

  // Proportional
  float pTerm = KP * error;

  // Integral with anti-windup (conditional integration)
  float proposedErrorSum = errorSum + (error * (CONTROL_LOOP_TIME_MS / 1000.0));
  float futureI = KI * proposedErrorSum;
  float dTermCheck = KD * ((error - lastError) / (CONTROL_LOOP_TIME_MS / 1000.0));
  
  // Feedforward - Convert to Tyre RPM for the model
  float activeTargetTyreRPM = activeTargetMotorRPM / GEAR_RATIO;
  float absTargetTyre = fabs(activeTargetTyreRPM);
  float ffMag = (absTargetTyre > 0.1) ? ((absTargetTyre / MOTOR_GAIN) + MOTOR_OFFSET) : 0;
  float ff = (activeTargetMotorRPM >= 0) ? ffMag : -ffMag;
  
  float estimatedOutput = pTerm + futureI + dTermCheck + ff;
  
  // Anti-windup: only integrate if not saturated or if integrating helps
  if (estimatedOutput >= -255 && estimatedOutput <= 255) {
    errorSum = proposedErrorSum;
  } else if (estimatedOutput > 255 && error < 0) {
    errorSum = proposedErrorSum;
  } else if (estimatedOutput < -255 && error > 0) {
    errorSum = proposedErrorSum;
  }
  
  errorSum = constrain(errorSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  
  float iTerm = KI * errorSum;

  // Derivative
  float dTerm = KD * (error - lastError);

  float pidOutput = pTerm + iTerm + dTerm;
  float totalControl = ff + pidOutput;

  int calculatedPWM = (int)totalControl;

  // Direction and PWM
  if (calculatedPWM >= 0) {
    digitalWrite(DIR_PIN, HIGH);
    currentPWM = calculatedPWM;
  } else {
    digitalWrite(DIR_PIN, LOW);
    currentPWM = abs(calculatedPWM);
  }

  if (currentPWM > 0 && currentPWM < MIN_PWM_OUTPUT) {
    currentPWM = MIN_PWM_OUTPUT;
  }

  currentPWM = constrain(currentPWM, PWM_MIN, PWM_MAX);
  ledcWrite(PWM_PIN, currentPWM);

  lastError = error;
}

// ================== LED ==================
void updateLED() {
  uint32_t color;

  if (canTimeoutDetected) {
    // Solid MAGENTA = No CAN commands
    color = statusLed.Color(255, 0, 255);
  } else if (fabs(finalTargetMotorRPM) > 0.1) {
    // Solid BLUE = Motor running
    color = statusLed.Color(0, 0, 255);
  } else {
    // Solid GREEN = Idle, receiving commands
    color = statusLed.Color(0, 255, 0);
  }

  statusLed.fill(color);
  statusLed.show();
}
