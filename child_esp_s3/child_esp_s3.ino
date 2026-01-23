/*
 * ESP32-S3 SLAVE MOTOR CONTROLLER - FINAL CLEAN VERSION
 * Change ONLY: CAN_ID_MOTOR, CAN_ID_FEEDBACK, and PID values for each wheel
 * 
 * FL: CAN_ID_MOTOR=0x124, CAN_ID_FEEDBACK=0x224, ENCODER_PPR=400
 * FR: CAN_ID_MOTOR=0x121, CAN_ID_FEEDBACK=0x221, ENCODER_PPR=400
 * BR: CAN_ID_MOTOR=0x122, CAN_ID_FEEDBACK=0x222, ENCODER_PPR=400
 * BL: CAN_ID_MOTOR=0x123, CAN_ID_FEEDBACK=0x223, ENCODER_PPR=600
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

// ================== CHANGE FOR EACH WHEEL ==================
#define CAN_ID_MOTOR     0x121   // Command ID
#define CAN_ID_FEEDBACK  0x221   // Feedback ID
#define ENCODER_PPR      400     // Encoder pulses per revolution

// PID Constants (FR values shown - CHANGE FOR EACH WHEEL)
#define KP 0.1418
#define KI 2.683
#define KD 0.0
#define MOTOR_GAIN   9.670
#define MOTOR_OFFSET 350.574
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
#define MIN_PWM_OUTPUT   20

#define RPM_SAMPLE_TIME_MS   50
#define CONTROL_LOOP_TIME_MS 50
#define EMA_ALPHA            0.3
#define RPM_DEADBAND         5.0
#define INTEGRAL_LIMIT       1000.0

#define MAX_CAN_SPEED 1000
#define MAX_RPM       2500
#define CAN_TIMEOUT_MS 500

// ================== CAN STRUCTS ==================
typedef struct {
  int16_t speed;
  int8_t  direction;
} __attribute__((packed)) motor_cmd_t;

typedef struct {
  float position;
  float velocity;
} __attribute__((packed)) motor_feedback_t;

// ================== VARIABLES ==================
float targetRPM = 0.0;
float currentRPM = 0.0;
float filteredRPM = 0.0;

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
void runPID();
float calculateRPM(int16_t count);
void updateLED();

// ================== SETUP ==================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

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

  Serial.println("✓ Slave Ready");
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

    float rpm = ((float)cmd.speed * MAX_RPM) / MAX_CAN_SPEED;
    if (cmd.direction == 0) rpm = -rpm;

    targetRPM = rpm;
    lastCANUpdate = millis();
  }
}

void sendFeedback() {
  motor_feedback_t fb;
  fb.position = globalEncoderTicks * TICKS_TO_RAD;
  fb.velocity = currentRPM * RPM_TO_RADS;

  twai_message_t msg = {};
  msg.identifier = CAN_ID_FEEDBACK;
  msg.data_length_code = sizeof(motor_feedback_t);
  memcpy(msg.data, &fb, sizeof(fb));

  twai_transmit(&msg, 0);
}

void checkCANTimeout() {
  if (millis() - lastCANUpdate > CAN_TIMEOUT_MS) {
    targetRPM = 0;
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
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

void measureRPM() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  globalEncoderTicks += count;  // Track absolute position
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

// ================== PID ==================
void runPID() {
  float error = targetRPM - currentRPM;

  // Stop condition - RESET integral when stopped
  if (fabs(targetRPM) < 0.1 && fabs(currentRPM) < 5.0) {
    ledcWrite(PWM_PIN, 0);
    currentPWM = 0;
    errorSum = 0;
    lastError = 0;
    return;
  }

  // ANTI-WINDUP: If actual RPM has opposite sign from target, reset integral
  // This prevents windup when motor is manually forced backwards
  if ((targetRPM > 0 && currentRPM < -50) || (targetRPM < 0 && currentRPM > 50)) {
    errorSum = 0;  // Reset integral when fighting external force
  }

  // ANTI-WINDUP: If error is huge (>500 RPM), likely external interference
  if (fabs(error) > 500) {
    errorSum *= 0.5;  // Decay integral quickly
  }

  if (fabs(error) < RPM_DEADBAND) error = 0;

  float p = KP * error;

  // Standard anti-windup during saturation
  bool saturated = (currentPWM >= PWM_MAX && error > 0) || 
                   (currentPWM <= -PWM_MAX && error < 0);

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

// ================== LED ==================
void updateLED() {
  uint32_t color;

  if (canTimeoutDetected) {
    // Solid MAGENTA = No CAN commands
    color = statusLed.Color(255, 0, 255);
  } else if (targetRPM != 0) {
    // Solid BLUE = Motor running
    color = statusLed.Color(0, 0, 255);
  } else {
    // Solid GREEN = Idle, receiving commands
    color = statusLed.Color(0, 255, 0);
  }

  statusLed.fill(color);
  statusLed.show();
}