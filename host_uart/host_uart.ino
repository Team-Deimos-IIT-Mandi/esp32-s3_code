/*
 * ESP32-S3 FRONT LEFT - MASTER + SLAVE WITH FULL RGB INDICATORS + UART
 * - Controls its own motor (FL)
 * - Receives UART commands from Jetson (separate from debug Serial)
 * - Sends CAN commands to other 3 slaves
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

#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define PWM_PIN       10
#define DIR_PIN       11
#define PCNT_UNIT     PCNT_UNIT_0
#define ENCODER_PPR   400

// ================== PID CONSTANTS (FL) ==================
#define KP 0.1369
#define KI 2.4825
#define KD 0.0
#define MOTOR_GAIN   10.4521
#define MOTOR_OFFSET 146.3001

// ================== CONTROL PARAMS ==================
#define PWM_FREQ             25000
#define PWM_RESOLUTION       8
#define PWM_MAX              255
#define MIN_PWM_OUTPUT       20
#define RPM_SAMPLE_TIME_MS   50
#define CONTROL_LOOP_TIME_MS 50
#define EMA_ALPHA            0.3
#define RPM_DEADBAND         5.0
#define INTEGRAL_LIMIT       1000.0
#define MAX_CAN_SPEED        1000
#define MAX_RPM              2500
#define UART_TIMEOUT_MS      500
#define CAN_FEEDBACK_TIMEOUT_MS 1000

const float MAX_RAD_S = 26.18;

// ================== SERIAL PROTOCOL ==================
#pragma pack(push, 1)
struct SerialCommand {
  uint8_t header;        // 0xA5
  float fl_vel;          // rad/s
  float rl_vel;          // rad/s
  float fr_vel;          // rad/s
  float rr_vel;          // rad/s
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
  int16_t speed;
  int8_t  direction;
} __attribute__((packed)) motor_cmd_t;

typedef struct {
  float position;
  float velocity;
} __attribute__((packed)) motor_feedback_t;

// ================== GLOBALS ==================
Adafruit_NeoPixel statusLed(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// My motor state
float myTargetRPM = 0.0;
float myCurrentRPM = 0.0;
float myFilteredRPM = 0.0;
int myCurrentPWM = 0;
volatile long myEncoderTicks = 0;
int16_t my_last_pulse_count = 0;

const float TICKS_TO_RAD = (2.0 * PI) / (ENCODER_PPR * 4.0);
const float RPM_TO_RADS = (2.0 * PI) / 60.0;

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
void runMyPID();
float calculateRPM(int16_t delta);
void processUARTCommands();
void readSlavesFeedback();
void sendUARTFeedback();
void sendCANCommand(uint32_t id, float rad_s);
void updateRGBStatus();

// ================== SETUP ==================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  // Initialize debug Serial (USB)
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32-S3 Front Left Master Starting ===");

  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  
  // Yellow = Initializing
  statusLed.fill(statusLed.Color(255, 255, 0));
  statusLed.show();
  delay(500);

  // Initialize UART for Jetson communication
  if (!initUART()) {
    Serial.println("ERROR: UART init failed!");
    statusLed.fill(statusLed.Color(255, 0, 255)); // Magenta = UART failed
    statusLed.show();
    while(1) delay(100);
  }
  Serial.println("UART initialized successfully");

  // Initialize CAN
  canInitOK = initCAN();
  if (!canInitOK) {
    Serial.println("ERROR: CAN init failed!");
    // Red = CAN Failed
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

  // Cyan = Ready, waiting for Jetson
  statusLed.fill(statusLed.Color(0, 255, 255));
  statusLed.show();
  
  Serial.println("=== System Ready - Waiting for Jetson ===");
  
  // Send initial feedback
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
    myTargetRPM = 0;
    sendCANCommand(CAN_ID_FR, 0.0);
    sendCANCommand(CAN_ID_BR, 0.0);
    sendCANCommand(CAN_ID_BL, 0.0);
  } else {
    if (!uartConnected) {
      Serial.println("Jetson connected via UART");
      uartConnected = true;
    }
  }

  // Check if motors are active
  motorsActive = (fabs(myTargetRPM) > 0.1);

  if (now - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureMyRPM();
    lastRPMTime += RPM_SAMPLE_TIME_MS;
  }

  if (now - lastControlTime >= CONTROL_LOOP_TIME_MS) {
    runMyPID();
    lastControlTime += CONTROL_LOOP_TIME_MS;
  }

  // Send feedback at 50Hz
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

  // Configure UART parameters
  if (uart_param_config(UART_NUM, &uart_config) != ESP_OK) {
    return false;
  }

  // Set UART pins
  if (uart_set_pin(UART_NUM, UART_TXD_PIN, UART_RXD_PIN, 
                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
    return false;
  }

  // Install UART driver
  if (uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0) != ESP_OK) {
    return false;
  }

  return true;
}

void processUARTCommands() {
  // Read available data from UART
  int len = uart_read_bytes(UART_NUM, uart_rx_buffer + uart_buffer_index, 
                            UART_BUF_SIZE - uart_buffer_index, 0);
  
  if (len > 0) {
    uart_buffer_index += len;
    
    // Look for valid command packet
    for (int i = 0; i <= uart_buffer_index - (int)sizeof(SerialCommand); i++) {
      if (uart_rx_buffer[i] == 0xA5) {
        // Check if we have enough bytes for complete packet
        if (i + sizeof(SerialCommand) <= uart_buffer_index) {
          // Check terminator
          if (uart_rx_buffer[i + sizeof(SerialCommand) - 1] == 0x5A) {
            // Valid packet found
            SerialCommand cmd;
            memcpy(&cmd, &uart_rx_buffer[i], sizeof(SerialCommand));
            
            lastUartCmdTime = millis();

            // Send commands to CAN slaves
            sendCANCommand(CAN_ID_FR, cmd.fr_vel);
            sendCANCommand(CAN_ID_BR, cmd.rr_vel);
            sendCANCommand(CAN_ID_BL, cmd.rl_vel);

            // Set my own motor target (convert rad/s to RPM)
            myTargetRPM = (cmd.fl_vel * 60.0) / (2.0 * PI);

            // Debug output
            if (fabs(cmd.fl_vel) > 0.01 || fabs(cmd.fr_vel) > 0.01) {
              Serial.printf("CMD: FL=%.2f RL=%.2f FR=%.2f RR=%.2f rad/s\n", 
                           cmd.fl_vel, cmd.rl_vel, cmd.fr_vel, cmd.rr_vel);
            }

            // Remove processed packet from buffer
            uart_buffer_index -= (i + sizeof(SerialCommand));
            memmove(uart_rx_buffer, uart_rx_buffer + i + sizeof(SerialCommand), uart_buffer_index);
            
            break; // Process one packet per call
          }
        }
      }
    }
    
    // Prevent buffer overflow - keep only last potential packet worth of data
    if (uart_buffer_index > UART_BUF_SIZE - (int)sizeof(SerialCommand)) {
      uart_buffer_index = 0;
      Serial.println("WARNING: UART buffer overflow, clearing");
    }
  }
}

void sendUARTFeedback() {
  SerialFeedback fb;
  fb.header = 0xA5;

  // My motor (Front Left)
  fb.fl_pos = myEncoderTicks * TICKS_TO_RAD;
  fb.fl_vel = myCurrentRPM * RPM_TO_RADS;

  // Slaves' feedback
  fb.rl_pos = pos_bl; 
  fb.rl_vel = vel_bl;
  fb.fr_pos = pos_fr; 
  fb.fr_vel = vel_fr;
  fb.rr_pos = pos_br; 
  fb.rr_vel = vel_br;

  fb.terminator = 0x5A;

  // Send via UART
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

void sendCANCommand(uint32_t id, float rad_s) {
  int16_t value = (int16_t)((rad_s / MAX_RAD_S) * MAX_CAN_SPEED);
  value = constrain(value, -MAX_CAN_SPEED, MAX_CAN_SPEED);

  motor_cmd_t cmd;
  if (value >= 0) {
    cmd.speed = value;
    cmd.direction = 1;
  } else {
    cmd.speed = abs(value);
    cmd.direction = 0;
  }

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
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
  my_last_pulse_count = 0;
}

void measureMyRPM() {
  int16_t current_count;
  pcnt_get_counter_value(PCNT_UNIT, &current_count);
  int16_t delta = current_count - my_last_pulse_count;
  my_last_pulse_count = current_count;
  myEncoderTicks += delta;

  float rawRPM = calculateRPM(delta);
  if (myFirstRPM) {
    myFilteredRPM = rawRPM;
    myFirstRPM = false;
  } else {
    myFilteredRPM = EMA_ALPHA * rawRPM + (1.0 - EMA_ALPHA) * myFilteredRPM;
  }
  myCurrentRPM = myFilteredRPM;
}

float calculateRPM(int16_t delta) {
  const int CPR = ENCODER_PPR * 4;
  return (float)delta * 60000.0 / (CPR * RPM_SAMPLE_TIME_MS);
}

void runMyPID() {
  float error = myTargetRPM - myCurrentRPM;

  // Stop condition - RESET integral when stopped
  if (fabs(myTargetRPM) < 0.1 && fabs(myCurrentRPM) < 5.0) {
    ledcWrite(PWM_PIN, 0);
    myCurrentPWM = 0;
    myErrorSum = 0;
    myLastError = 0;
    return;
  }

  // ANTI-WINDUP: If actual RPM has opposite sign from target, reset integral
  if ((myTargetRPM > 0 && myCurrentRPM < -50) || (myTargetRPM < 0 && myCurrentRPM > 50)) {
    myErrorSum = 0;
  }

  // ANTI-WINDUP: If error is huge, likely external interference
  if (fabs(error) > 500) {
    myErrorSum *= 0.5;
  }

  if (fabs(error) < RPM_DEADBAND) error = 0;

  float p = KP * error;

  bool saturated = (myCurrentPWM >= PWM_MAX && error > 0) ||
                   (myCurrentPWM <= -PWM_MAX && error < 0);

  if (!saturated) {
    myErrorSum += error * (CONTROL_LOOP_TIME_MS / 1000.0);
    myErrorSum = constrain(myErrorSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  float i = KI * myErrorSum;
  float d = KD * (error - myLastError);

  float ffMag = (fabs(myTargetRPM) + MOTOR_OFFSET) / MOTOR_GAIN;
  float ff = (myTargetRPM >= 0) ? ffMag : -ffMag;

  float control = ff + p + i + d;

  if (control >= 0) {
    digitalWrite(DIR_PIN, LOW);
    myCurrentPWM = (int)control;
  } else {
    digitalWrite(DIR_PIN, HIGH);
    myCurrentPWM = (int)fabs(control);
  }

  if (myCurrentPWM > 0 && myCurrentPWM < MIN_PWM_OUTPUT) {
    myCurrentPWM = MIN_PWM_OUTPUT;
  }

  myCurrentPWM = constrain(myCurrentPWM, 0, PWM_MAX);
  ledcWrite(PWM_PIN, myCurrentPWM);
  myLastError = error;
}

// ================== RGB STATUS INDICATORS ==================
void updateRGBStatus() {
  unsigned long now = millis();
  uint32_t color1, color2;

  // LED 1: Jetson Communication Status
  if (!uartConnected) {
    // Magenta = No Jetson connection
    color1 = statusLed.Color(255, 0, 255);
  } else if (motorsActive) {
    // Blue = Jetson connected, motors active
    color1 = statusLed.Color(0, 0, 255);
  } else {
    // Green = Jetson connected, idle
    color1 = statusLed.Color(0, 255, 0);
  }

  // LED 2: CAN Slaves Status
  bool allSlavesOK = (now - lastFeedback_FR < CAN_FEEDBACK_TIMEOUT_MS) &&
                     (now - lastFeedback_BR < CAN_FEEDBACK_TIMEOUT_MS) &&
                     (now - lastFeedback_BL < CAN_FEEDBACK_TIMEOUT_MS);
  
  if (!allSlavesOK && uartConnected) {
    // Orange = Some slaves not responding
    color2 = statusLed.Color(255, 100, 0);
  } else if (motorsActive) {
    // Cyan = All systems active
    color2 = statusLed.Color(0, 255, 255);
  } else if (uartConnected) {
    // Green = All systems ready
    color2 = statusLed.Color(0, 255, 0);
  } else {
    // White = Waiting for Jetson
    color2 = statusLed.Color(100, 100, 100);
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
/*

## Key Changes Made:

1. **Added UART driver** (`UART_NUM_1` on GPIO 17/18)
2. **Separated debug Serial** (USB) from UART (Jetson communication)
3. **Changed all `Serial.available()` → `uart_read_bytes()`**
4. **Changed all `Serial.write()` → `uart_write_bytes()`**
5. **Added UART buffer management** to handle packet synchronization
6. **Kept all your features**: RGB LEDs, PID with anti-windup, CAN communication, timeouts
7. **Added debug output** to USB Serial for monitoring

## Wiring:

ESP32 GPIO 17 (TX) → Jetson RX (e.g., /dev/ttyTHS0 RX)
ESP32 GPIO 18 (RX) → Jetson TX (e.g., /dev/ttyTHS0 TX)
GND → GND (CRITICAL!)*/