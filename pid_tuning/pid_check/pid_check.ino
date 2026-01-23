/*
 * ESP32-S3 Hybrid Control (Feedforward + PID + Smooth Target)
 * FIX: Ramps the TARGET RPM instead of the PWM Output.
 * Result: No Jerk, No Oscillation.
 */

#include "driver/pcnt.h"

// --- PIN DEFINITIONS ---
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define PWM_PIN 10
#define DIR_PIN 11

// --- SPECIFICATIONS ---
#define ENCODER_PPR 400
#define GEAR_RATIO 13.7        
#define PCNT_UNIT PCNT_UNIT_0

// --- PWM CONFIG ---
#define PWM_FREQ 25000
#define PWM_RESOLUTION 8

// --- TIMING ---
#define RPM_SAMPLE_TIME_MS 50
#define CONTROL_LOOP_TIME_MS 50

// --- MOTOR MODEL ---
#define MOTOR_GAIN 0.7632      
#define MOTOR_OFFSET 13.41     

// --- PID GAINS (Default: Safe start values) ---
float kp = 0.3;    
float ki = 1;    
float kd = 0.01;   

// --- LIMITS ---
#define PWM_MIN 0
#define PWM_MAX 255
#define INTEGRAL_LIMIT 1000.0  

// --- MOTION PROFILE (SMOOTHNESS) ---
// How many RPM to change per loop (50ms).
// 2.0 Motor RPM per loop = Approx 40 Motor RPM per second (Very Smooth)
// Increase this number to make it faster. Decrease to make it smoother.
float accelerationStep = 150.0; 

// --- EMA FILTER ---
#define EMA_ALPHA 0.3

// --- GLOBAL VARIABLES ---
float finalTargetMotorRPM = 0.0; // The final destination (e.g., 20)
float activeTargetRPM = 0.0;     // The moving target the PID actually chases
float currentMotorRPM = 0.0;
float filteredMotorRPM = 0.0;
int currentPWM = 0;

// PID Variables
float errorSum = 0.0;
float lastError = 0.0;
unsigned long lastControlTime = 0;
unsigned long lastRPMTime = 0;

int16_t lastPulseCount = 0;
bool firstRPMReading = true;
String inputString = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(1000);
  
  Serial.println("=== ESP32 Smooth Motion Control ===");
  Serial.println("Commands:");
  Serial.println("  [Number] -> Target Tyre RPM");
  Serial.println("  P/I/D[Value] -> Tune Gains");
  Serial.println("  A[Value] -> Set Acceleration (e.g. A10)");
  
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);
  
  initPCNT();
  inputString.reserve(20);
  lastRPMTime = millis();
  lastControlTime = millis();
  
  printGains();
}

void loop() {
  unsigned long currentTime = millis();
  checkSerialInput();
  
  if (currentTime - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureRPM();
    lastRPMTime = currentTime;
  }
  
  if (currentTime - lastControlTime >= CONTROL_LOOP_TIME_MS) {
    // 1. Update the Moving Target (Motion Profiling)
    updateSetpoint();
    
    // 2. Run PID on the Moving Target
    runPIDControl();
    
    displayStatus();
    lastControlTime = currentTime;
  }
}

void checkSerialInput() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) processSerialInput();
    } else {
      inputString += inChar;
    }
  }
}

void processSerialInput() {
  inputString.trim();
  if (inputString.length() == 0) return;

  char cmd = toupper(inputString.charAt(0));
  
  if (cmd == 'P' || cmd == 'I' || cmd == 'D') {
    String valueStr = inputString.substring(1);
    float value = valueStr.toFloat();
    if (cmd == 'P') kp = value;
    else if (cmd == 'I') ki = value;
    else if (cmd == 'D') kd = value;
    
    Serial.print(">>> TUNING: "); Serial.print(cmd); Serial.print("="); Serial.println(value);
    errorSum = 0.0; lastError = 0.0;
    
  } else if (cmd == 'A') {
    // Set Acceleration Rate
    String valueStr = inputString.substring(1);
    accelerationStep = valueStr.toFloat();
    Serial.print(">>> ACCEL RATE: "); Serial.println(accelerationStep);

  } else if (isdigit(cmd) || cmd == '-') {
    float newTyreTarget = inputString.toFloat();
    
    // We only update the FINAL destination.
    // The "activeTargetRPM" will slowly travel there in the loop.
    finalTargetMotorRPM = newTyreTarget * GEAR_RATIO;
    
    Serial.print(">>> NEW DESTINATION: "); Serial.println(newTyreTarget);
  }
  inputString = "";
}

// --- NEW FUNCTION: Moves the target slowly ---
void updateSetpoint() {
  // If we are below target, step up
  if (activeTargetRPM < finalTargetMotorRPM) {
    activeTargetRPM += accelerationStep;
    // Don't overshoot
    if (activeTargetRPM > finalTargetMotorRPM) activeTargetRPM = finalTargetMotorRPM;
  } 
  // If we are above target, step down
  else if (activeTargetRPM > finalTargetMotorRPM) {
    activeTargetRPM -= accelerationStep;
    // Don't overshoot
    if (activeTargetRPM < finalTargetMotorRPM) activeTargetRPM = finalTargetMotorRPM;
  }
}

void measureRPM() {
  int16_t currentCount;
  pcnt_get_counter_value(PCNT_UNIT, &currentCount);
  pcnt_counter_clear(PCNT_UNIT);
  
  int16_t deltaCounts = currentCount;
  float rawRPM = calculateRPM(deltaCounts, RPM_SAMPLE_TIME_MS);
  
  if (firstRPMReading) {
    filteredMotorRPM = rawRPM;
    firstRPMReading = false;
  } else {
    filteredMotorRPM = (EMA_ALPHA * rawRPM) + ((1.0 - EMA_ALPHA) * filteredMotorRPM);
  }
  currentMotorRPM = filteredMotorRPM;
}

void runPIDControl() {
  // PID now chases "activeTargetRPM" (the moving one), NOT finalTarget
  float error = activeTargetRPM - currentMotorRPM;
  
  // 1. Proportional
  float pTerm = kp * error;
  
  // 2. Integral with Clamping
  float proposedErrorSum = errorSum + (error * (CONTROL_LOOP_TIME_MS / 1000.0));
  float futureI = ki * proposedErrorSum;
  float dTermCheck = 0; 
  if (CONTROL_LOOP_TIME_MS > 0) dTermCheck = kd * ((error - lastError) / (CONTROL_LOOP_TIME_MS / 1000.0));
  
  float absTarget = fabs(activeTargetRPM);
  float targetTyreForFF = absTarget / GEAR_RATIO;
  float ffMag = (targetTyreForFF > 0.1) ? ((targetTyreForFF / MOTOR_GAIN) + MOTOR_OFFSET) : 0;
  float ff = (activeTargetRPM >= 0) ? ffMag : -ffMag;
  
  float estimatedOutput = pTerm + futureI + dTermCheck + ff;
  
  if (estimatedOutput >= -255 && estimatedOutput <= 255) {
      errorSum = proposedErrorSum;
  } else if (estimatedOutput > 255 && error < 0) {
      errorSum = proposedErrorSum;
  } else if (estimatedOutput < -255 && error > 0) {
      errorSum = proposedErrorSum;
  }
  
  if (errorSum > INTEGRAL_LIMIT) errorSum = INTEGRAL_LIMIT;
  else if (errorSum < -INTEGRAL_LIMIT) errorSum = -INTEGRAL_LIMIT;
  
  float iTerm = ki * errorSum;
  
  // 3. Derivative
  float dTerm = 0.0;
  if (CONTROL_LOOP_TIME_MS > 0) {
    float errorRate = (error - lastError) / (CONTROL_LOOP_TIME_MS / 1000.0);
    dTerm = kd * errorRate;
  }
  
  float pidOutput = pTerm + iTerm + dTerm;
  
  // 4. Feedforward using the MOVING target
  float feedforward = (activeTargetRPM >= 0) ? ffMag : -ffMag;
  float rawPidOutput = feedforward + pidOutput;
  int calculatedPWM = (int)rawPidOutput;

  // NO OUTPUT RAMPING HERE! The target is already ramped.

  // 5. Actuate
  if (calculatedPWM >= 0) {
    digitalWrite(DIR_PIN, HIGH);
    currentPWM = calculatedPWM;
  } else {
    digitalWrite(DIR_PIN, LOW);
    currentPWM = abs(calculatedPWM);
  }
  
  if (currentPWM > PWM_MAX) currentPWM = PWM_MAX;
  if (currentPWM < PWM_MIN) currentPWM = PWM_MIN;
  
  ledcWrite(PWM_PIN, currentPWM);
  lastError = error;
}

void displayStatus() {
  Serial.print("Target:");
  Serial.print(activeTargetRPM / GEAR_RATIO, 2); 
  Serial.print(" Actual:");
  Serial.print(currentMotorRPM / GEAR_RATIO, 2); 
  Serial.print(" PWM:");
  Serial.println(currentPWM);
}

void printGains() {
  Serial.print("Gains -> Kp:"); Serial.print(kp);
  Serial.print(" Ki:"); Serial.print(ki);
  Serial.print(" Kd:"); Serial.println(kd);
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