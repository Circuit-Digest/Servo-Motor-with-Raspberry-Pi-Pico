# Servo Motor Control with Raspberry Pi Pico using PWM in MicroPython
====================================================

[![Raspberry Pi Pico](https://img.shields.io/badge/Raspberry%20Pi-C51A4A?style=for-the-badge&logo=Raspberry-Pi&logoColor=white)](https://www.raspberrypi.org/) 
[![MicroPython](https://img.shields.io/badge/MicroPython-2E8B57?style=for-the-badge&logo=micropython&logoColor=white)](https://micropython.org/) 
[![PWM](https://img.shields.io/badge/PWM-FF6B35?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTMgMTJIMjFNMyA2SDE4TTMgMThIMTUiIHN0cm9rZT0iI0ZGRkZGRiIgc3Ryb2tlLXdpZHRoPSIyIiBzdHJva2UtbGluZWNhcD0icm91bmQiLz4KPC9zdmc+)](https://en.wikipedia.org/wiki/Pulse-width_modulation) 
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT) 
[![CircuitDigest](https://img.shields.io/badge/Tutorial-CircuitDigest-blue?style=for-the-badge)](https://circuitdigest.com/microcontroller-projects/control-a-servo-motor-with-raspberry-pi-pico-using-pwm-in-micropython)

A **Precision Servo Motor Control System** using Raspberry Pi Pico and MicroPython with PWM signals for accurate angle positioning from 0° to 180°. Perfect for robotics, automation, and mechatronics projects requiring precise motor control.

![Raspberry Pi Pico Servo Control](https://circuitdigest.com/sites/default/files/projectimage_mic/Raspberry-Pi-Pico-Servo-Motor-Control.jpg)

🚀 Features
-----------

- **Precise Angle Control** - 0° to 180° positioning with high accuracy
- **PWM Signal Generation** - 50Hz frequency with variable duty cycle
- **MicroPython Programming** - Easy-to-understand Python-like syntax
- **Real-Time Control** - Instant response to position commands
- **Low Power Consumption** - Efficient power management for battery projects
- **Multiple Servo Support** - Control up to 8 servos simultaneously
- **Smooth Movement** - Configurable sweep and positioning functions
- **Educational Friendly** - Perfect for learning embedded systems

🛠️ Hardware Requirements
-------------------------

### Core Components

- **Raspberry Pi Pico** (1x) - RP2040 microcontroller board
- **SG90 Servo Motor** (1x or more) - 180° micro servo (or compatible)
- **Breadboard** - For circuit assembly
- **Jumper Wires** - Male-to-male connections
- **USB Cable** - For programming and power

### Power Supply Options

- **5V External Supply** - For multiple servos or high-torque applications
- **VBUS Pin Power** - Direct from USB (sufficient for SG90)
- **Battery Pack** - For portable/mobile applications

### Optional Components

- **Logic Level Shifter** - For 5V servo compatibility (if needed)
- **Capacitors** - Power supply filtering (100µF recommended)
- **Potentiometer** - Manual angle control input
- **LEDs** - Visual feedback indicators

📐 Circuit Diagram
------------------

```
Raspberry Pi Pico GPIO Connections:
┌────────────┬──────────────┬───────────────────────┐
│ Pico Pin   │ Servo Wire   │ Function              │
├────────────┼──────────────┼───────────────────────┤
│ GPIO 0     │ Yellow/White │ PWM Signal Control    │
│ VBUS (5V)  │ Red          │ Power Supply          │
│ GND        │ Brown/Black  │ Common Ground         │
└────────────┴──────────────┴───────────────────────┘

Alternative Power Connection:
For external 5V supply:
5V Supply (+) → Servo Red Wire
5V Supply (-) → Pico GND + Servo Brown Wire
Pico GPIO 0 → Servo Yellow Wire
```

🔧 Installation
---------------

### 1. MicroPython Setup

Download and install MicroPython firmware on your Pico:
```bash
# Download latest MicroPython UF2 file from:
# https://micropython.org/download/rp2-pico/

# Hold BOOTSEL button while connecting USB
# Drag and drop UF2 file to RPI-RP2 drive
```

### 2. IDE Installation

Install Thonny IDE for MicroPython development:
```bash
# Download from: https://thonny.org/
# Configure interpreter: Tools → Options → Interpreter
# Select "MicroPython (Raspberry Pi Pico)"
```

### 3. Hardware Assembly

1. **Connect Servo to Pico:**
   - Red wire → VBUS (Pin 40)
   - Brown/Black wire → GND (Pin 38)
   - Yellow/White wire → GPIO 0 (Pin 1)

2. **Verify Connections:**
   - Ensure secure connections
   - Check power supply polarity
   - Confirm PWM pin assignment

### 4. Code Upload

```bash
git clone https://github.com/Circuit-Digest/Pico-Servo-Control-PWM.git
cd Pico-Servo-Control-PWM
```

Open `main.py` in Thonny IDE and save to Raspberry Pi Pico.

🎯 Usage
--------

### 1. Basic Servo Control

Upload and run the basic control script:
```python
from machine import Pin, PWM
from time import sleep

# Initialize PWM on GPIO 0 with 50Hz frequency
pwm = PWM(Pin(0))
pwm.freq(50)

# Move servo to 0°, 90°, and 180°
def setServoCycle(position):
    pwm.duty_u16(position)
    sleep(0.01)

# Continuous sweep demonstration
while True:
    for pos in range(1000, 9000, 50):  # 0° to 180°
        setServoCycle(pos)
    for pos in range(9000, 1000, -50): # 180° to 0°
        setServoCycle(pos)
```

### 2. Angle-Based Control

For precise angle positioning:
```python
def set_angle(angle):
    # Convert angle (0-180°) to duty cycle (1000-9000)
    duty = int(1000 + (angle / 180) * 8000)
    pwm.duty_u16(duty)
    sleep(0.02)

# Position servo at specific angles
set_angle(0)    # 0 degrees
set_angle(90)   # 90 degrees  
set_angle(180)  # 180 degrees
```

### 3. Multiple Servo Control

Control multiple servos simultaneously:
```python
servo1 = PWM(Pin(0))
servo2 = PWM(Pin(1))
servo1.freq(50)
servo2.freq(50)

# Synchronized movement
for angle in range(0, 181, 5):
    duty = int(1000 + (angle / 180) * 8000)
    servo1.duty_u16(duty)
    servo2.duty_u16(9000 - duty)  # Counter-rotation
    sleep(0.1)
```

📁 Code Structure
-----------------

```
Pico-Servo-Control-PWM/
├── Code/
│   ├── main.py                      # Basic servo control
│   ├── servo_sweep.py               # Continuous sweep demo
│   ├── angle_control.py             # Precise angle positioning
│   ├── multiple_servos.py           # Multi-servo control
│   └── servo_library.py             # Servo control class
├── Circuit_Diagrams/
│   ├── Basic_Connection.png         # Simple servo wiring
│   ├── Multiple_Servos.png          # Multi-servo setup
│   └── External_Power.png           # External power configuration
├── Examples/
│   ├── servo_potentiometer.py       # Manual control with pot
│   ├── servo_bluetooth.py           # Wireless control
│   └── servo_sensor.py              # Sensor-triggered movement
└── README.md
```

🔧 Troubleshooting
------------------

### Common Issues

**Servo Not Moving**

- Check PWM frequency (must be 50Hz for most servos)
- Verify duty cycle range (1000-9000 for 0°-180°)
- Ensure adequate power supply (5V, minimum 500mA)
- Confirm GPIO pin supports PWM output

**Erratic Movement**

- Check power supply stability and current capacity
- Add 100µF capacitor across power rails
- Ensure clean connections without loose wires
- Verify servo is not mechanically obstructed

**Limited Range**

- Adjust duty cycle values (try 500-2500 microseconds)
- Check servo specifications for pulse width requirements
- Calibrate min/max positions in code
- Some servos may have <180° range

**Power Issues**

- Use external 5V supply for multiple servos
- Check VBUS can provide sufficient current
- Add bypass capacitors near servo connections
- Consider logic level shifter for 5V servos

### PWM Calculation Guide

```python
# PWM Theory for Servo Control:
# Frequency = 50Hz (20ms period)
# Pulse Width: 1ms (0°) to 2ms (180°)
# Duty Cycle = (Pulse Width / Period) × 100%

# For 0°: 1ms/20ms = 5% = 3276 (in 16-bit PWM)
# For 180°: 2ms/20ms = 10% = 6553 (in 16-bit PWM)
# MicroPython scales to 0-65535 range

def pulse_width_to_duty(pulse_us):
    return int((pulse_us / 20000) * 65535)

# Standard servo ranges:
min_duty = pulse_width_to_duty(1000)  # 1ms = 0°
max_duty = pulse_width_to_duty(2000)  # 2ms = 180°
```

📱 Applications
---------------

- **Robotics Projects** - Robot arms, walking robots, pan-tilt mechanisms
- **Home Automation** - Automated blinds, door locks, valve control
- **RC Vehicles** - Steering mechanisms, camera gimbals
- **Educational Projects** - STEM learning, mechatronics demonstrations
- **Art Installations** - Kinetic sculptures, interactive displays
- **Industrial Control** - Positioning systems, automated machinery
- **Surveillance Systems** - Camera positioning, scanning mechanisms
- **Model Making** - Animated models, remote-controlled projects

🔮 Future Enhancements
----------------------

- [ ] **Smooth Motion Control** - Acceleration/deceleration curves
- [ ] **Servo Calibration Tool** - Automatic min/max position detection
- [ ] **Wireless Control** - WiFi/Bluetooth remote operation
- [ ] **Sensor Integration** - Automatic positioning based on sensors
- [ ] **Multi-Axis Control** - Coordinated movement of multiple servos
- [ ] **Position Feedback** - Closed-loop control with encoders
- [ ] **GUI Control Interface** - Computer-based servo controller
- [ ] **Voice Command Control** - Speech-activated positioning

🏗️ Technical Specifications
----------------------------

| Parameter              | Value                    |
|------------------------|--------------------------|
| Operating Voltage      | 3.3V (Logic) / 5V (Servo)|
| PWM Frequency          | 50Hz                     |
| PWM Resolution         | 16-bit (0-65535)        |
| Servo Range            | 0° to 180°              |
| Pulse Width Range      | 1ms to 2ms              |
| Update Rate            | Up to 50Hz               |
| Multiple Servo Support | Limited by GPIO pins    |
| Power Consumption      | ~10mA (Pico) + Servo    |
| Operating Temperature  | 0°C to 50°C             |
| Response Time          | ~20ms                    |

🔬 PWM Theory and Implementation
-------------------------------

### Pulse Width Modulation Basics

PWM controls servo position by varying the width of periodic pulses:

**Key Parameters:**
- **Frequency:** 50Hz (20ms period) - Standard for analog servos
- **Pulse Width:** 1-2ms determines angle (1ms=0°, 1.5ms=90°, 2ms=180°)
- **Duty Cycle:** Percentage of time signal is HIGH during each period

### MicroPython PWM Implementation

```python
# PWM duty calculation for servo control
def angle_to_duty(angle):
    """Convert angle (0-180°) to PWM duty cycle value"""
    # Map 0-180° to 1000-2000 microseconds pulse width
    pulse_width = 1000 + (angle / 180.0) * 1000
    
    # Convert to 16-bit duty cycle (0-65535)
    duty_cycle = int((pulse_width / 20000.0) * 65535)
    return duty_cycle

# Usage example
servo = PWM(Pin(0))
servo.freq(50)
servo.duty_u16(angle_to_duty(90))  # Set to 90°
```

### Advanced Control Techniques

```python
class ServoController:
    def __init__(self, pin, min_us=1000, max_us=2000):
        self.servo = PWM(Pin(pin))
        self.servo.freq(50)
        self.min_us = min_us
        self.max_us = max_us
    
    def set_angle(self, angle):
        """Set servo angle with bounds checking"""
        angle = max(0, min(180, angle))
        pulse_us = self.min_us + (angle/180) * (self.max_us - self.min_us)
        duty = int((pulse_us / 20000) * 65535)
        self.servo.duty_u16(duty)
    
    def sweep(self, start=0, end=180, step=5, delay=0.1):
        """Smooth sweep between angles"""
        for angle in range(start, end + step, step):
            self.set_angle(angle)
            sleep(delay)
```

🔗 Related Projects
-------------------

- **🤖 Robotics**: [Raspberry Pi Pico Robot Projects](https://circuitdigest.com/raspberry-pi-pico-projects)
- **🏠 Automation**: [Home Automation with Pico](https://circuitdigest.com/microcontroller-projects/home-automation-projects)
- **📡 IoT Control**: [Pico IoT Projects](https://circuitdigest.com/internet-of-things-iot-projects)
- **🎓 Learning**: [MicroPython Tutorials](https://circuitdigest.com/micropython-tutorials)

📊 Performance Characteristics
------------------------------

### Servo Response Times

| Movement Range | Response Time | Accuracy |
|----------------|---------------|----------|
| 0° to 30°      | ~150ms        | ±1°      |
| 0° to 90°      | ~300ms        | ±1°      |
| 0° to 180°     | ~600ms        | ±2°      |
| Small steps    | ~20ms         | ±0.5°    |

### Power Consumption Analysis

| Configuration     | Current Draw | Notes                |
|-------------------|--------------|----------------------|
| Single SG90       | ~100-200mA   | During movement      |
| Single SG90 (idle)| ~5-10mA      | Holding position     |
| Multiple servos   | N × 200mA    | External supply req. |
| Pico only         | ~20mA        | Without servos       |

⚠️ Safety and Best Practices
----------------------------

- **Power Supply:** Use adequate current capacity for servo count
- **Voltage Levels:** Ensure compatibility between Pico (3.3V) and servo (5V)
- **Mechanical Limits:** Avoid forcing servo beyond its physical range
- **Heat Management:** Allow cooling time for continuous operation
- **Code Safety:** Include bounds checking for angle values
- **Wiring:** Use appropriate wire gauge for current requirements

💡 Pro Tips for Better Performance
----------------------------------

### Optimization Techniques

1. **Smooth Motion:** Implement acceleration/deceleration curves
2. **Power Management:** Turn off PWM when not moving
3. **Calibration:** Measure actual min/max pulse widths for your servo
4. **Noise Reduction:** Add bypass capacitors and proper grounding

### Code Optimization

```python
# Efficient multi-servo control
class MultiServoController:
    def __init__(self, pins):
        self.servos = [PWM(Pin(pin)) for pin in pins]
        for servo in self.servos:
            servo.freq(50)
    
    def set_positions(self, angles):
        """Set all servo positions simultaneously"""
        for servo, angle in zip(self.servos, angles):
            duty = self.angle_to_duty(angle)
            servo.duty_u16(duty)
    
    def disable_all(self):
        """Turn off all PWM signals to save power"""
        for servo in self.servos:
            servo.deinit()
```

**Built with ❤️ by [Circuit Digest](https://circuitdigest.com/)**

*Empowering makers with precision control solutions*

---

### Keywords

`raspberry pi pico servo control` `micropython pwm servo` `servo motor pico` `pwm servo control` `micropython servo library` `pico servo motor` `servo angle control` `raspberry pi servo` `micropython projects` `servo motor control python` `pico pwm tutorial` `servo robotics projects`
