# EchoRover 🚗🔊

An **Arduino Due robot project** that combines:
- Ultrasonic distance sensing (HC-SR04, interrupt-driven)
- Servo-based head sweep
- Dual DC motors with **non-blocking software PWM** on L293D
- 7-segment multiplexed display
- Traffic-light style LEDs and buzzer feedback

Designed as a **fully non-blocking control loop** (no `delay()`), making it a solid reference for interrupt handling, multiplexed displays, and smooth robot control.

---

## 📸 Features
- **Ultrasonic sensor** (HC-SR04)  
  - Distance measured with interrupt-driven echo timing.  
  - Median-of-three filter for stable readings.  

- **Servo scanner**  
  - Smooth 20°–110° sweep with small timed increments.  
  - Non-blocking, step-by-step movement.  

- **7-segment display (3-digit)**  
  - Multiplexed in software at ~2 kHz.  
  - Displays live distance in cm (0–999).  

- **LEDs & buzzer**  
  - Green = safe (>70 cm)  
  - Yellow = caution (30–70 cm)  
  - Red + buzzer = obstacle (<30 cm)  

- **Dual DC motors (L293D)**  
  - Software PWM @ 200 Hz on EN pins.  
  - FAST, SLOW, or STOP depending on zone.  
  - In RED zone, one motor is randomly stopped for a “swerve” effect.  

- **Non-blocking design**  
  - Uses `micros()` and `millis()` scheduling.  
  - No blocking delays—everything runs in parallel.  

---

## ⚡ Hardware
- **Microcontroller:** Arduino Due (3.3V logic)  
- **Motor driver:** L293D  
- **Motors:** 2 × DC motors  
- **Sensor:** HC-SR04 ultrasonic (**ECHO must be level-shifted to 3.3V**)  
- **Servo:** SG90 / micro servo  
- **Display:** 3-digit 7-segment, common-cathode  
- **LEDs:** Red, Yellow, Green  
- **Buzzer:** Active buzzer (HIGH = ON)  
- **Power:** 9V battery (VIN) + regulated 5V rail for peripherals  
- **Common ground:** Tie all grounds together (Due GND, motor driver GND, sensor/buzzer/LED GND).

---

## 🛠 Wiring Overview

### Ultrasonic
| Signal | Pin |
|---|---|
| TRIG | D2 |
| ECHO | D12 *(via divider/level shifter to 3.3V)* |

### Servo
| Signal | Pin |
|---|---|
| Control | D3 |

### 3-Digit 7-Segment (Common Cathode)
| Role | Pins |
|---|---|
| Segments A–G | D4..D10 |
| Digits (cathodes) | D11, D13, D22 |

### LEDs & Buzzer
| Device | Pin |
|---|---|
| LED_R | D23 |
| LED_Y | D24 |
| LED_G | D25 |
| Buzzer (active) | D26 |

### L293D Motors
| Motor | EN | IN1 | IN2 |
|---|---|---|---|
| M1 | D33 | D32 | D31 |
| M2 | D27 | D28 | D29 |

> **Tip:** If a motor spins backward versus your expectations, flip its `dir` in setup or swap the motor leads.

---

## 📊 Behavior Zones
| Distance (cm) | Zone   | LEDs | Motors                             | Buzzer |
|---|---|---|---|---|
| > 70          | GREEN  | G    | Both **FAST**                      | OFF |
| 30–70         | YELLOW | Y    | Both **SLOW**                      | OFF |
| ≤ 30          | RED    | R    | One motor randomly **STOPPED**, other **SLOW** | ON |

---

## 📂 Repo Structure
~~~text
EchoRover/
├── src/
│   └── main.ino          # Full Arduino code
├── docs/
│   └── wiring-diagram.png (optional schematic)
└── README.md             # This file
~~~

---

## 🚀 Getting Started

1) **Clone**
~~~bash
git clone https://github.com/<your-username>/EchoRover.git
cd EchoRover
~~~

2) **Open & build**
- Open `src/main.ino` in Arduino IDE (or PlatformIO).
- Select **Arduino Due (Programming Port)** as the board.
- Compile & upload.

3) **Power**
- Provide 9V to VIN (or suitable source) for the Due.  
- Provide appropriate motor power (can share VIN if within specs).  
- **Common GND is mandatory** across Due, L293D, and peripherals.

---

## 🧠 Design Notes
- **Zero blocking delays:** all tasks scheduled with `millis()`/`micros()`.
- **Interrupt-driven ultrasonic:** accurate echo timing with CHANGE ISR.
- **Median-of-three filter:** smooths distance spikes/outliers.
- **Software PWM @ 200 Hz:** on L293D EN pins; 0%/100% special-cased.
- **Servo sweep smoothing:** 1° steps every 15 ms for natural motion.
- **7-seg multiplexing:** ~2 kHz total refresh; minimal flicker.
- **Randomized evasive action:** in red zone, one motor stops to bias a turn.

---

## 🧪 Troubleshooting

- **Servo jittery or stalls**  
  - Ensure servo has a stable 5V rail (not from USB).  
  - Keep grounds common; avoid long thin wires for servo power.

- **Display too dim**  
  - Check current-limiting resistors on segments; multiplexing reduces duty per digit.  
  - Reduce `DIGIT_PERIOD_US` for a higher refresh (mind CPU load).

- **Ultrasonic reads 0 or 999**  
  - Verify level shifting on ECHO.  
  - Confirm TRIG pulse wiring and that nothing blocks the sensor.

- **Motors don’t move / are weak**  
  - Confirm separate motor supply if needed; L293D drops voltage.  
  - Try `SPEED_FAST = 255`, `SPEED_SLOW ~ 120–160`.  
  - Ensure EN pins are actually toggling (scope or LED check).

---

## 📸 Demo
*(Add a photo or video link here once your robot runs.)*

---

## 🔮 Future Improvements
- Add side IR sensors for better avoidance.  
- Replace L293D with TB6612FNG (lower losses, better current).  
- Add IMU for heading stabilization.  
- Integrate Bluetooth/Wi‑Fi teleop mode.  
- Add PID speed control with encoders.

