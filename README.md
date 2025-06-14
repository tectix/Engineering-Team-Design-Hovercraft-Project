# Autonomous Navigation Hovercraft

An Arduino-based autonomous hovercraft designed for competitive robotics with advanced navigation capabilities, dual-fan propulsion system, and real-time obstacle avoidance.

## Project Overview

This project implements a fully autonomous hovercraft capable of navigating complex environments using sensor fusion, real-time path planning, and precision control systems. Built for robotics competitions with strict size and weight constraints while maintaining high performance and reliability.

## Key Features

- **Autonomous Navigation**: State-machine based navigation with 100Hz sensor fusion
- **Obstacle Avoidance**: Real-time scanning with servo-controlled ultrasonic sensor
- **Dual-Fan Propulsion**: Independent lift and thrust control for precise maneuvering
- **Competition Ready**: Meets strict 48cm dimension and 1.5kg weight limits
- **High Precision**: ±5° heading accuracy with IMU-based stabilization
- **Extended Runtime**: 12-15 minute operation on single LiPo battery

## Hardware Components

### Control System
- **Arduino Uno R3**: Central microcontroller (ATmega328P @ 16MHz)
- **32KB Flash Memory**: Program storage with 2KB SRAM
- **14 Digital I/O Pins**: 6 PWM channels for motor control

### Sensor Array
- **MPU6050 IMU**: 6-axis gyroscope and accelerometer
  - ±2000°/s gyroscope range
  - ±4g accelerometer range
  - 100Hz update rate via I2C
- **HC-SR04 Ultrasonic**: Distance measurement (2cm - 400cm range)
  - ±3mm accuracy with 15° beam angle
  - 10Hz scanning rate
- **SG90 Servo Motor**: Sensor positioning and directional control
  - 180° rotation with ±1° accuracy
  - 0.1s/60° positioning speed

### Propulsion System
- **Lift Fan**: 120mm diameter, 800g static thrust
  - 12V DC motor @ 2000 RPM
  - 2.5A maximum current draw
- **Thrust Fan**: 80mm diameter, 450g static thrust
  - 12V DC motor @ 2400 RPM
  - 1.8A maximum current draw

### Power Management
- **LiPo Battery**: 2200mAh 3S (11.1V) lithium polymer
  - 25C continuous discharge rate
  - 185g weight, 12-15 minute runtime

## Performance Specifications

| Metric | Value | Description |
|--------|--------|-------------|
| Navigation Accuracy | ±5° | Heading precision with IMU fusion |
| Obstacle Detection | 35cm | Reliable ultrasonic range |
| Sensor Update Rate | 100Hz | IMU sampling frequency |
| Servo Response Time | 500ms | Complete positioning cycle |
| Total Weight | <1.5kg | Including battery and frame |
| Max Dimensions | 48cm | Competition size constraint |

## 🛠Setup Instructions

### Prerequisites
- Arduino IDE 1.8.x or higher
- Required libraries:
  - `Wire.h` (I2C communication)
  - `Servo.h` (Servo control)
  - `MPU6050` library by Electronic Cats
  - `NewPing` library for HC-SR04

### Hardware Assembly
1. **Frame Construction**: Assemble lightweight frame within 48cm constraint
2. **Fan Mounting**: Install lift fan (bottom) and thrust fan (rear) with proper airflow
3. **Electronics Integration**: Mount Arduino, sensors, and battery with proper weight distribution
4. **Wiring**: Connect components according to pin diagram (see `/docs/wiring.md`)

### Software Installation
```bash
# Clone the repository
git clone https://github.com/yourusername/autonomous-hovercraft.git
cd autonomous-hovercraft

# Install required libraries through Arduino IDE Library Manager
# Or use the provided installation script
./scripts/install_libraries.sh
```

### Configuration
1. **Calibrate IMU**: Run `calibration/imu_calibrate.ino` 
2. **Test Motors**: Use `tests/motor_test.ino` to verify fan operation
3. **Sensor Verification**: Run `tests/sensor_check.ino` for all sensors

## Usage

### Basic Operation
```cpp
// Upload main navigation code
// File: src/autonomous_navigation.ino

void setup() {
  initializeSensors();
  calibrateIMU();
  setNavigationMode(AUTONOMOUS);
}

void loop() {
  updateSensorData();
  processNavigation();
  executeMovement();
}
```

### Competition Mode
- Power on the hovercraft
- Wait for sensor initialization (LED indicator)
- Place in starting position
- System automatically begins navigation sequence

### Manual Override
- Connect via serial monitor (9600 baud)
- Send commands: `FORWARD`, `REVERSE`, `LEFT`, `RIGHT`, `STOP`
- Emergency stop: Send `EMERGENCY` or power cycle

---
