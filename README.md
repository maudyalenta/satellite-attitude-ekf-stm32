# Satellite Attitude Determination Using MEMS & EKF on STM32L4

An embedded implementation of an Extended Kalman Filter (EKF) sensor fusion algorithm for satellite Attitude Determination and Control Systems (ADCS), deployed on an ultra-low-power STM32L4 series microcontroller.

---

## 📌 Project Overview
Small satellite missions require precise attitude estimation under strict power and computational constraints. This project implements a real-time sensor fusion pipeline that fuses multi-axis inertial and magnetic measurements into reliable quaternion and Euler angle representations, mitigating sensor bias and noise on bare-metal hardware.

- **Author:** Maudy
- **Role:** Undergraduate Researcher & Embedded Firmware Developer
- **Target Microcontroller:** STM32L4xx (ARM Cortex-M4)
- **Target Sensors:** ICM-42688 (6-Axis IMU) & BNO055 (Absolute Orientation / Magnetometer)
- **Primary Algorithm:** Extended Kalman Filter (EKF) Sensor Fusion

---

## 📁 Repository Structure & Modular Architecture

```text
├── Inc/
│   ├── attitude_kalman_extended.h  # EKF state estimator and covariance definitions
│   ├── bno055.h / bno055_stm32.h   # BNO055 sensor driver and HAL interface
│   ├── icm42688.h / *_stm32.h      # ICM-42688 high-performance IMU driver
│   ├── icm42688_calib.h            # Gyro/accel bias calibration routines
│   ├── mag_calib.h                 # Magnetometer hard/soft-iron calibration
│   ├── sensor_manager.h            # Multi-sensor data acquisition scheduler
│   ├── imu_norm.h                  # Vector normalization routines
│   ├── imu2quaternion.h            # Kinematic measurement conversions
│   ├── quaternion.h                # Quaternion algebra and operations
│   ├── quat2euler.h                # Quaternion to Euler angle transformations
│   ├── quaternion2imu.h            # Reference frame projection utilities
│   ├── webserial_print.h           # Real-time telemetry output interface
│   ├── system_init.h / main.h      # System clock and hardware peripherals setup
│   └── stm32l4xx_*.h               # STM32 HAL configuration and interrupt headers
│
├── Src/
│   ├── attitude_kalman_extended.c  # EKF propagation and measurement update steps
│   ├── bno055.c                    # Low-level I2C/SPI register transactions for BNO055
│   ├── icm42688.c                  # Low-level I2C/SPI register transactions for ICM-42688
│   ├── icm42688_calib.c            # Bias estimation and zero-rate calibration
│   ├── mag_calib.c                 # Magnetic distortion compensation algorithms
│   ├── sensor_manager.c            # Telemetry polling and data synchronization
│   ├── imu_norm.c                  # Sensor magnitude normalization implementations
│   ├── imu2quaternion.c            # Attitude initialization from vectors
│   ├── quaternion.c                # Quaternion mathematical operations
│   ├── quat2euler.c                # Roll, pitch, and yaw conversion algorithms
│   ├── quaternion2imu.c            # Gravity/magnetic reference projections
│   ├── webserial_print.c           # Serial streaming implementation for monitoring
│   ├── system_init.c / main.c      # Main program execution loop and timing
│   ├── stm32l4xx_*.c               # Peripheral initialization and MSP configurations
│   └── syscalls.c / sysmem.c       # Low-level runtime system calls and memory management
│
└── README.md
