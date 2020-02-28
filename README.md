# imu-platform
Device orientation determination project.

Project is based on STM32F407VET6 controller, which gets data from LSM6DS3 and LS303C ST accelerometer, gyroscope and magnetometer. Data is processed with Sebastian Madgwick's [algorithm](http://x-io.co.uk/res/doc/madgwick_internal_report.pdf) and quaternion describing device orientation is produced. This quaternion and other data transferred to GCS (ground control system) via USART/nRF24L01. GCS plots acceleration, angle velocity, magnetometer data plots and rotates 3D-model similar to orientation of system, so you can see how is your system oriented.

## Structure
1. **board** folder

   Includes source code and needed files for STM32F407VET6 controller (used on "blackboard" developing plate).
   Source code is in **_src_** folder. Includes drivers, library functions for Madgwick's algorithm and global "tasks".
   
   Board source code is written in Eclipse IDE, so it will be easier to use it to program the board.
   
2. **gcs** folder (made by [dll31](https://github.com/dll31)).

   Used to plot graphs of transferred values and 3D-model which orientation is similar to orientation of system. Realized on Python 3.

3. **calibration** folder

   Stores calibration values for accelerometer and magnetometer (arrays of data used to produce them).

4. **message_definitions** folder

   Generated C source code describing MAVLink messages.

Global interaction model: STM32 transfers data via USART or SPI by nRF24L01 to computer running GCS. Data is transferred using [MAVLink](https://mavlink.io/en/) protocol.

## Installation

Installation instructions are written for Linux.

1. Clone git project

```bash
git clone https://github.com/korr237i/imu-platform.git
cd imu-platform/
git submodule init && git submodule update
```

2. Install PyMAVLink for GCS

```bash
./install_pymavlink.sh
```

 To run GCS simply execute:

```bash
python3 main.py
```

