# ESP32-S3 Mini – Single Motor PID Controller (Stealth Safety Edition)
**Team Deimos | IIT Mandi**

This repository contains the firmware for a robust, high-performance DC motor controller based on the **ESP32-S3**. It utilizes the ESP32's dedicated hardware peripherals (TWAI/CAN, PCNT, LEDC) to drive a motor with precise PID speed control while maintaining strict safety protocols.

## 🌟 Key Features

* **CAN Bus Control:** Receives target speed and direction via the TWAI (Two-Wire Automotive Interface) driver.
* **Advanced PID Algorithm:**
    * Includes **Feedforward (FF)** for rapid response.
    * **Integral Anti-Windup** to prevent overshoot.
    * **EMA Filtering** to smooth noisy encoder data.
    * **Deadband** management to eliminate jitter at zero speed.
* **Stealth Safety System:**
    * **Visual Status:** A smart RGB LED system that remains **OFF** during normal operation (Stealth) and only lights up to indicate specific faults or braking states.
    * **Auto-Stop:** Automatically cuts motor power if CAN communication is lost for >500ms.
    * **Stall/Disconnect Detection:** Intelligently detects if the encoder is disconnected or the motor is stalled.

## 🔌 Hardware Pinout

| Peripheral | Function | GPIO Pin | Note |
| :--- | :--- | :--- | :--- |
| **CAN Bus** | TX | `GPIO 1` | Connect to CAN Transceiver TX |
| | RX | `GPIO 2` | Connect to CAN Transceiver RX |
| **Encoder** | Phase A | `GPIO 4` | Hardware Pulse Counter (PCNT) |
| | Phase B | `GPIO 5` | Hardware Pulse Counter (PCNT) |
| **Motor** | PWM | `GPIO 10` | 25kHz High Frequency PWM |
| | Direction | `GPIO 11` | High/Low logic |
| **Status** | RGB LED | `GPIO 6` | WS2812B / NeoPixel |

## 🧠 Control Logic

The system operates on a 50ms control loop. It reads the encoder tick count, converts it to RPM, applies an Exponential Moving Average (EMA) filter, and feeds it into the PID controller.



**Equation:**
$$Output = (K_p \cdot e) + (K_i \cdot \int e) + (K_d \cdot \Delta e) + FeedForward$$

* **Target RPM:** Derived from CAN message.
* **Current RPM:** Derived from PCNT (Encoder).
* **Feedforward:** Uses `MOTOR_GAIN` and `MOTOR_OFFSET` to estimate required power, reducing the work needed by the PID terms.

## 📡 CAN Communication Protocol

The controller listens for standard CAN frames with **ID `0x123`**.

#CAN package structure
typedef struct {
  int16_t speed;     // Magnitude (0 to 1000)
  int8_t  direction; // 1 = Forward, 0 = Reverse
} __attribute__((packed)) motor_cmd_t;

Speed Mapping: A CAN speed value of 1000 maps to MAX_RPM (2500 RPM).

Timeout: If no message is received for 500ms, the motor acts as if Target RPM is 0 (Safety Stop).

🚨 Status LED Codes (Stealth Mode)
Unlike traditional controllers, the LED remains OFF during normal operation to save power and reduce visual clutter. It only blinks during specific events:
Color,Pattern,State,Action Required
⚫ OFF,Static,Normal,System operating nominally.
🟣 Purple,Blinking,CAN Timeout,Check CAN wiring or master controller status.
🟠 Yellow,Blinking,Encoder Fault,Encoder disconnected or motor stalled. Check wiring.
🟢 Lime,Blinking,Active Braking,Motor is actively fighting inertia to stop. Info only.

⚙️ Configuration
You can tune the system by modifying the macros at the top of main.cpp:

PID Gains: Change KP, KI, KD to tune response.

Motor Char: Adjust MOTOR_GAIN and MOTOR_OFFSET based on your specific motor's voltage/RPM curve.

Limits: Update MAX_RPM and PWM_MAX to match your mechanical constraints.

🛠 Usage
Install Libraries: Ensure Adafruit_NeoPixel is installed in your Arduino Library Manager.

Select Board: Select "ESP32S3 Dev Module" in Arduino IDE.

USB Mode: Ensure "USB CDC On Boot" is Enabled if you need Serial debugging.

Flash: Connect USB and upload.
