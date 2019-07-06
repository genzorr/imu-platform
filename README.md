# STM32F4-IMU
Device orientation determination project

The idea of the project is to get data from IMU device (MPU9255) and use Sebastian Madgwick's [algorithm](http://x-io.co.uk/res/doc/madgwick_internal_report.pdf) to determine device's orientation. Project based on STM32F407VET6 board.

## Folders
Source folder includes needed files for STM32F4 and source code of the project.

GCS folder - ground control system that allows you to plot graphs and device's orientation (made by [dll31](https://github.com/dll31)).
