/*
 * ESP32-S3 FRONT LEFT - MASTER + SLAVE WITH SMOOTH MOTOR RPM CONTROL (REFACTORED)
 * - Controls its own motor (FL) with smooth ramping on Motor RPM scale
 * - Receives UART commands from Jetson (rad/s)
 * - Sends CAN commands to other 3 slaves (tyre RPM)
 * - Shows clear status via RGB LEDs
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include <Adafruit_NeoPixel.h>

// ================== UART CONFIGURATION ==================
#define UART_NUM UART_NUM_1
#define UART_TXD_PIN 9  // ESP32 TX -> Jetson RX
#define UART_RXD_PIN 12  // ESP32 RX -> Jetson TX
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024

// ================== CAN IDS ==================
#define CAN_ID_FR 0x121  // Front Right
#define CAN_ID_BR 0x122  // Back Right
#define CAN_ID_BL 0x123  // Back Left

#define FB_ID_FR 0x221
#define FB_ID_BR 0x222
#define FB_ID_BL 0x223

// ================== PINS ==================
#define RGB_LED_PIN     6
#define NUM_PIXELS      2
#define LED_BRIGHTNESS  80

#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2

#define ENCODER_A_PIN 5
#define ENCODER_B_PIN 4
#define PWM_PIN       10
#define DIR_PIN       11
#define PCNT_UNIT     PCNT_UNIT_0
#define ENCODER_PPR   400
#define GEAR_RATIO    13.7

// ================== PID CONSTANTS (FL) ==================
#define KP 0.3
#define KI 1.0
#define KD 0.01
#define MOTOR_GAIN   0.7632
#define MOTOR_OFFSET 13.41

// Motion Profile
#define ACCELERATION_STEP 140.0  // Motor RPM change per control loop (~1s to full speed)

// ================== CONTROL PARAMS ==================
#define PWM_FREQ             25000
#define PWM_RESOLUTION       8
#define PWM_MAX              255
#define PWM_MIN              0
#define MIN_PWM_OUTPUT       20

#define RPM_SAMPLE_TIME_MS   50
#define CONTROL_LOOP_TIME_MS 50
#define EMA_ALPHA            0.3
#define RPM_DEADBAND         5.0
#define INTEGRAL_LIMIT       1000.0

#define MAX_RPM              182.48  // Max tyre RPM
#define UART_TIMEOUT_MS      500
#define CAN_FEEDBACK_TIMEOUT_MS 1000

const float RPM_TO_RADS = (2.0 * PI) / 60.0;
const float RADS_TO_RPM = 60.0 / (2.0 * PI);

// ================== SERIAL PROTOCOL ==================
#pragma pack(push, 1)
struct SerialCommand {
  uint8_t header;        // 0xA5
  float fl_vel;          // rad/s (tyre)
  float rl_vel;          // rad/s (tyre)
  float fr_vel;          // rad/s (tyre)
  float rr_vel;          // rad/s (tyre)
  uint8_t terminator;    // 0x5A
};

struct SerialFeedback {
  uint8_t header;        // 0xA5
  float fl_pos; float fl_vel;
  float rl_pos; float rl_vel;
  float fr_pos; float fr_vel;
  float rr_pos; float rr_vel;
  uint8_t terminator;    // 0x5A
};
#pragma pack(pop)

typedef struct {
  float target_rpm;  // Tyre RPM
} __attribute__((packed)) motor_cmd_t;

typedef struct {
  float position;
  float velocity;
} __attribute__((packed)) motor_feedback_t;

// ================== GLOBALS ==================
Adafruit_NeoPixel statusLed(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// My motor state - NOW IN MOTOR RPM
float myFinalTargetMotorRPM = 0.0;    // Final destination (Motor RPM)
float myActiveTargetMotorRPM = 0.0;   // Moving target (ramped, Motor RPM)
float myCurrentMotorRPM = 0.0;
float myFilteredMotorRPM = 0.0;
int myCurrentPWM = 0;
volatile long myEncoderTicks = 0;

const float TICKS_TO_RAD = (2.0 * PI) / (ENCODER_PPR * 4.0);

float myErrorSum = 0.0;
float myLastError = 0.0;
bool myFirstRPM = true;

// Slaves' feedback
float pos_fr=0, vel_fr=0;
float pos_br=0, vel_br=0;
float pos_bl=0, vel_bl=0;

unsigned long lastRPMTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastTelemTime = 0;
unsigned long lastUartCmdTime = 0;

// Slave feedback tracking
unsigned long lastFeedback_FR = 0;
unsigned long lastFeedback_BR = 0;
unsigned long lastFeedback_BL = 0;

// Status flags
bool uartConnected = false;
bool canInitOK = false;
bool motorsActive = false;

// UART buffer
static uint8_t uart_rx_buffer[UART_BUF_SIZE];
static int uart_buffer_index = 0;

// ================== FUNCTION DECLARATIONS ==================
bool initCAN();
bool initUART();
void initPCNT();
void measureMyRPM();
void updateMySetpoint();
void runMyPID();
float calculateRPM(int16_t delta);
void processUARTCommands();
void readSlavesFeedback();
void sendUARTFeedback();
void sendCANCommand(uint32_t id, float tyre_rpm);
void updateRGBStatus();

// ================== SETUP ==================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);

  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32-S3 Front Left Master - Smooth Motor Control (Refactored) ===");

  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  
  statusLed.fill(statusLed.Color(255, 255, 0));
  statusLed.show();
  delay(500);

  if (!initUART()) {
    Serial.println("ERROR: UART init failed!");
    statusLed.fill(statusLed.Color(255, 0, 255));
    statusLed.show();
    while(1) delay(100);
  }
  Serial.println("UART initialized successfully");

  canInitOK = initCAN();
  if (!canInitOK) {
    Serial.println("ERROR: CAN init failed!");
    statusLed.fill(statusLed.Color(255, 0, 0));
    statusLed.show();
    while(1) delay(100);
  }
  Serial.println("CAN initialized successfully");

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);

  initPCNT();
  Serial.println("Encoder initialized");

  lastRPMTime = millis();
  lastControlTime = millis();
  lastTelemTime = millis();
  lastUartCmdTime = millis();
  lastFeedback_FR = millis();
  lastFeedback_BR = millis();
  lastFeedback_BL = millis();

  statusLed.fill(statusLed.Color(0, 255, 255));
  statusLed.show();
  
  Serial.println("=== System Ready - Waiting for Jetson ===");
  
  delay(100);
  sendUARTFeedback();
}

// ================== MAIN LOOP ==================
void loop() {
  unsigned long now = millis();

  processUARTCommands();
  readSlavesFeedback();

  // Safety timeout
  if (now - lastUartCmdTime > UART_TIMEOUT_MS) {
    if (uartConnected) {
      Serial.println("WARNING: UART timeout - stopping motors");
      uartConnected = false;
    }
    myFinalTargetMotorRPM = 0;
    sendCANCommand(CAN_ID_FR, 0.0);
    sendCANCommand(CAN_ID_BR, 0.0);
    sendCANCommand(CAN_ID_BL, 0.0);
  } else {
    if (!uartConnected) {
      Serial.println("Jetson connected via UART");
      uartConnected = true;
    }
  }

  motorsActive = (fabs(myFinalTargetMotorRPM) > 0.1);

  if (now - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureMyRPM();
    lastRPMTime += RPM_SAMPLE_TIME_MS;
  }

  if (now - lastControlTime >= CONTROL_LOOP_TIME_MS) {
    updateMySetpoint();  // Ramp the target
    runMyPID();
    lastControlTime += CONTROL_LOOP_TIME_MS;
  }

  if (now - lastTelemTime >= 20) {
    sendUARTFeedback();
    lastTelemTime = now;
  }

  updateRGBStatus();
}

// ================== UART FUNCTIONS ==================
bool initUART() {
  const uart_config_t uart_config = {
    .baud_rate = UART_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  if (uart_param_config(UART_NUM, &uart_config) != ESP_OK) {
    return false;
  }

  if (uart_set_pin(UART_NUM, UART_TXD_PIN, UART_RXD_PIN, 
                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
    return false;
  }

  if (uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0) != ESP_OK) {
    return false;
  }

  return true;
}

void processUARTCommands() {
  int len = uart_read_bytes(UART_NUM, uart_rx_buffer + uart_buffer_index, 
                            UART_BUF_SIZE - uart_buffer_index, 0);
  
  if (len > 0) {
    uart_buffer_index += len;
    
    for (int i = 0; i <= uart_buffer_index - (int)sizeof(SerialCommand); i++) {
      if (uart_rx_buffer[i] == 0xA5) {
        if (i + sizeof(SerialCommand) <= uart_buffer_index) {
          if (uart_rx_buffer[i + sizeof(SerialCommand) - 1] == 0x5A) {
            SerialCommand cmd;
            memcpy(&cmd, &uart_rx_buffer[i], sizeof(SerialCommand));
            
            lastUartCmdTime = millis();

            // Convert rad/s to tyre RPM and send to CAN slaves (UNCHANGED)
            float fr_tyre_rpm = cmd.fr_vel * RADS_TO_RPM;
            float rr_tyre_rpm = cmd.rr_vel * RADS_TO_RPM;
            float rl_tyre_rpm = cmd.rl_vel * RADS_TO_RPM;

            sendCANCommand(CAN_ID_FR, fr_tyre_rpm);
            sendCANCommand(CAN_ID_BR, rr_tyre_rpm);
            sendCANCommand(CAN_ID_BL, rl_tyre_rpm);

            // Set my own final target - Convert rad/s directly to MOTOR RPM
            myFinalTargetMotorRPM = cmd.fl_vel * RADS_TO_RPM * GEAR_RATIO;

            if (fabs(cmd.fl_vel) > 0.01 || fabs(cmd.fr_vel) > 0.01) {
              Serial.printf("CMD: FL=%.2f RL=%.2f FR=%.2f RR=%.2f rad/s\n", 
                           cmd.fl_vel, cmd.rl_vel, cmd.fr_vel, cmd.rr_vel);
            }

            uart_buffer_index -= (i + sizeof(SerialCommand));
            memmove(uart_rx_buffer, uart_rx_buffer + i + sizeof(SerialCommand), uart_buffer_index);
            
            break;
          }
        }
      }
    }
    
    if (uart_buffer_index > UART_BUF_SIZE - (int)sizeof(SerialCommand)) {
      uart_buffer_index = 0;
      Serial.println("WARNING: UART buffer overflow, clearing");
    }
  }
}

void sendUARTFeedback() {
  SerialFeedback fb;
  fb.header = 0xA5;

  // My motor (Front Left) - convert motor RPM to tyre velocity (rad/s)
  fb.fl_pos = myEncoderTicks * TICKS_TO_RAD;
  fb.fl_vel = (myCurrentMotorRPM / GEAR_RATIO) * RPM_TO_RADS;

  // Slaves' feedback (already in tyre rad/s)
  fb.rl_pos = pos_bl; 
  fb.rl_vel = vel_bl;
  fb.fr_pos = pos_fr; 
  fb.fr_vel = vel_fr;
  fb.rr_pos = pos_br; 
  fb.rr_vel = vel_br;

  fb.terminator = 0x5A;

  uart_write_bytes(UART_NUM, (const char*)&fb, sizeof(SerialFeedback));
}

// ================== CAN FUNCTIONS ==================
bool initCAN() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) == ESP_OK) {
    if (twai_start() == ESP_OK) {
      return true;
    }
  }
  return false;
}

void sendCANCommand(uint32_t id, float tyre_rpm) {
  motor_cmd_t cmd;
  cmd.target_rpm = tyre_rpm;

  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = sizeof(motor_cmd_t);
  memcpy(msg.data, &cmd, sizeof(cmd));

  twai_transmit(&msg, 0);
}

void readSlavesFeedback() {
  twai_message_t msg;
  unsigned long now = millis();

  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.data_length_code == 8) {
      float p, v;
      memcpy(&p, &msg.data[0], 4);
      memcpy(&v, &msg.data[4], 4);

      switch(msg.identifier) {
        case FB_ID_FR: 
          pos_fr = p; vel_fr = v; 
          lastFeedback_FR = now;
          break;
        case FB_ID_BR: 
          pos_br = p; vel_br = v; 
          lastFeedback_BR = now;
          break;
        case FB_ID_BL: 
          pos_bl = p; vel_bl = v; 
          lastFeedback_BL = now;
          break;
      }
    }
  }
}

// ================== MY MOTOR CONTROL ==================
void initPCNT() {
  pcnt_config_t ch0 = {
    .pulse_gpio_num = ENCODER_A_PIN,
    .ctrl_gpio_num = ENCODER_B_PIN,
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_DEC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT,
    .channel = PCNT_CHANNEL_0
  };
  pcnt_unit_config(&ch0);
  
  pcnt_config_t ch1 = {
    .pulse_gpio_num = ENCODER_B_PIN,
    .ctrl_gpio_num = ENCODER_A_PIN,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_REVERSE,
    .pos_mode = PCNT_COUNT_DEC,
    .neg_mode = PCNT_COUNT_INC,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT,
    .channel = PCNT_CHANNEL_1
  };
  pcnt_unit_config(&ch1);
  
  pcnt_set_filter_value(PCNT_UNIT, 100);
  pcnt_filter_enable(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

void measureMyRPM() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  myEncoderTicks += count;
  pcnt_counter_clear(PCNT_UNIT);

  float rawRPM = calculateRPM(count);
  
  if (myFirstRPM) {
    myFilteredMotorRPM = rawRPM;
    myFirstRPM = false;
  } else {
    myFilteredMotorRPM = EMA_ALPHA * rawRPM + (1.0 - EMA_ALPHA) * myFilteredMotorRPM;
  }
  myCurrentMotorRPM = myFilteredMotorRPM;
}

float calculateRPM(int16_t delta) {
  const int CPR = ENCODER_PPR * 4;
  return (float)delta * 60000.0 / (CPR * RPM_SAMPLE_TIME_MS);
}

void updateMySetpoint() {
  // Ramp the active target towards final target (now in Motor RPM)
  if (myActiveTargetMotorRPM < myFinalTargetMotorRPM) {
    myActiveTargetMotorRPM += ACCELERATION_STEP;
    if (myActiveTargetMotorRPM > myFinalTargetMotorRPM) {
      myActiveTargetMotorRPM = myFinalTargetMotorRPM;
    }
  } else if (myActiveTargetMotorRPM > myFinalTargetMotorRPM) {
    myActiveTargetMotorRPM -= ACCELERATION_STEP;
    if (myActiveTargetMotorRPM < myFinalTargetMotorRPM) {
      myActiveTargetMotorRPM = myFinalTargetMotorRPM;
    }
  }
}

void runMyPID() {
  // Calculate error directly in Motor RPM
  float error = myActiveTargetMotorRPM - myCurrentMotorRPM;

  // Stop condition
  if (fabs(myActiveTargetMotorRPM) < 0.1 && fabs(myCurrentMotorRPM) < 5.0) {
    ledcWrite(PWM_PIN, 0);
    myCurrentPWM = 0;
    myErrorSum = 0;
    myLastError = 0;
    return;
  }

  if (fabs(error) < RPM_DEADBAND) error = 0;

  // Proportional
  float pTerm = KP * error;

  // Integral with anti-windup
  float proposedErrorSum = myErrorSum + (error * (CONTROL_LOOP_TIME_MS / 1000.0));
  float futureI = KI * proposedErrorSum;
  float dTermCheck = KD * ((error - myLastError) / (CONTROL_LOOP_TIME_MS / 1000.0));
  
  // Feedforward - Convert to Tyre RPM for the model
  float myActiveTargetTyreRPM = myActiveTargetMotorRPM / GEAR_RATIO;
  float absTargetTyre = fabs(myActiveTargetTyreRPM);
  float ffMag = (absTargetTyre > 0.1) ? ((absTargetTyre / MOTOR_GAIN) + MOTOR_OFFSET) : 0;
  float ff = (myActiveTargetMotorRPM >= 0) ? ffMag : -ffMag;
  
  float estimatedOutput = pTerm + futureI + dTermCheck + ff;
  
  // Conditional integration
  if (estimatedOutput >= -255 && estimatedOutput <= 255) {
    myErrorSum = proposedErrorSum;
  } else if (estimatedOutput > 255 && error < 0) {
    myErrorSum = proposedErrorSum;
  } else if (estimatedOutput < -255 && error > 0) {
    myErrorSum = proposedErrorSum;
  }
  
  myErrorSum = constrain(myErrorSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  
  float iTerm = KI * myErrorSum;
  float dTerm = KD * (error - myLastError);

  float pidOutput = pTerm + iTerm + dTerm;
  float totalControl = ff + pidOutput;

  int calculatedPWM = (int)totalControl;

  // Direction and PWM
  if (calculatedPWM >= 0) {
    digitalWrite(DIR_PIN, HIGH);
    myCurrentPWM = calculatedPWM;
  } else {
    digitalWrite(DIR_PIN, LOW);
    myCurrentPWM = abs(calculatedPWM);
  }

  if (myCurrentPWM > 0 && myCurrentPWM < MIN_PWM_OUTPUT) {
    myCurrentPWM = MIN_PWM_OUTPUT;
  }

  myCurrentPWM = constrain(myCurrentPWM, PWM_MIN, PWM_MAX);
  ledcWrite(PWM_PIN, myCurrentPWM);
  myLastError = error;
}

// ================== RGB STATUS INDICATORS ==================
void updateRGBStatus() {
  unsigned long now = millis();
  uint32_t color1, color2;

  // LED 1: Jetson Communication Status
  if (!uartConnected) {
    color1 = statusLed.Color(255, 0, 255);  // Magenta
  } else if (motorsActive) {
    color1 = statusLed.Color(0, 0, 255);    // Blue
  } else {
    color1 = statusLed.Color(0, 255, 0);    // Green
  }

  // LED 2: CAN Slaves Status
  bool allSlavesOK = (now - lastFeedback_FR < CAN_FEEDBACK_TIMEOUT_MS) &&
                     (now - lastFeedback_BR < CAN_FEEDBACK_TIMEOUT_MS) &&
                     (now - lastFeedback_BL < CAN_FEEDBACK_TIMEOUT_MS);
  
  if (!allSlavesOK && uartConnected) {
    color2 = statusLed.Color(255, 100, 0);  // Orange
  } else if (motorsActive) {
    color2 = statusLed.Color(0, 255, 255);  // Cyan
  } else if (uartConnected) {
    color2 = statusLed.Color(0, 255, 0);    // Green
  } else {
    color2 = statusLed.Color(100, 100, 100); // White/Gray
  }

  statusLed.setPixelColor(0, color1);
  statusLed.setPixelColor(1, color2);
  statusLed.show();
}

/*
 * ==================== RGB LED GUIDE ====================
 * 
 * LED 1 (First LED) - JETSON STATUS:
 * 🟣 MAGENTA      = No Jetson connection (UART timeout)
 * 🟢 GREEN        = Jetson connected, motors idle
 * 🔵 BLUE         = Jetson connected, motors running
 * 
 * LED 2 (Second LED) - SYSTEM STATUS:
 * ⚪ WHITE/GRAY   = Waiting for Jetson
 * 🟢 GREEN        = All slaves responding, system ready
 * 🔵 CYAN         = All systems active
 * 🟠 ORANGE       = Some CAN slaves not responding
 * 
 * STARTUP SEQUENCE:
 * 🟡 YELLOW (both) = Initializing
 * 🟣 MAGENTA (both)= UART init failed (stuck here)
 * 🔴 RED (both)    = CAN init failed (stuck here)
 * 🔵 CYAN (both)   = Ready, waiting for Jetson
 * 
 * ========================================================
 */
