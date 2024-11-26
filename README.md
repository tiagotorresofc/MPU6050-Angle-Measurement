# **MPU6050 Angle Measurement with FreeRTOS**

This project demonstrates real-time angle measurement using an MPU6050 sensor connected to a Raspberry Pi Pico. It leverages FreeRTOS to manage tasks efficiently and features raw and filtered angle calculations for accurate tilt monitoring.

---

## **Features**
- **Raw Angle Calculation**: Direct computation from accelerometer data.
- **Filtered Angle Calculation**: Enhanced stability using an Exponential Moving Average (EMA) filter.
- **FreeRTOS Integration**: Efficient task management for real-time performance.
- **Clear Output Format**: Displays angles in a user-friendly format for easy analysis.

---

## **Applications**
- Balancing platforms (e.g., seesaws, robotic arms).
- Motion analysis for embedded systems.
- Real-time tilt monitoring.

---

## **Getting Started**

### **Hardware Requirements**
- Raspberry Pi Pico
- MPU6050 sensor
- I2C wiring (connect SDA to GPIO4 and SCL to GPIO5)
- USB cable for programming and power

### **Software Requirements**
- [CMake](https://cmake.org/)
- [FreeRTOS](https://freertos.org/)
- Pico SDK

---
