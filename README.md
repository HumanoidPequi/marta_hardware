# Firmware for Teensy 4.0 on PlatformIO

## 📌 Overview
This project contains the embedded firmware for **Teensy 4.0**, developed for the **Pequi Mecânico** humanoid robotics team. The code has been modularized to facilitate maintenance and system scalability.

## 📁 Project Structure

```
/marta_hardware/
│-- platformio.ini
│-- src/
│   │-- main.cpp
│   │-- imu_manager.cpp
│   │-- dynamixel_manager.cpp
│   │-- ros_comm.cpp
│   │-- sync_rw.cpp
│   │-- config.cpp
│-- include/
│   │-- imu_manager.h
│   │-- dynamixel_manager.h
│   │-- ros_comm.h
│   │-- sync_rw.h
│   │-- config.h
│-- lib/  (custom libraries, if needed)
│-- test/ (unit tests, if needed)
```

### 🛠️ Dependencies
This project uses the following libraries:
- **Dynamixel2Arduino** (for Dynamixel motor communication)
- **SparkFun BNO080** (for IMU sensors)
- **rosserial_arduino** (for ROS communication)

## 🚀 How to Compile and Upload the Code
1. Install [PlatformIO](https://platformio.org/)
2. Clone this repository:
   ```sh
   git clone git@github.com:HumanoidPequi/marta_hardware.git
   cd marta_hardware
   ```
3. Compile and upload the firmware:
   ```sh
   pio run --target upload
   ```
4. Monitor the serial output:
   ```sh
   pio device monitor
   ```

## 📌 Contribution
Follow best development practices to contribute to the project:
- Use **specific branches** for new features.
- Keep the code **modular and well-documented**.
- Test before opening a **pull request**.

