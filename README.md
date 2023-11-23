# HMC5883L_to_TM4C123G
Interfacing the HMC5883L 3-Axis Digital Compass with the Tiva TM4C123G LaunchPad
## Introduction
This document outlines the steps taken to initialize and configure the HMC5883L 3-Axis Digital Compass for use with the Tiva TM4C123G microcontroller. The process includes initializing the I2C communication protocol and configuring the compass module for accurate heading readings.
## I2C Initialization
Inter-Integrated Circuit (I2C) is a serial communication protocol that allows multiple slave devices to be controlled by a master device over a single bus. To interface the Tiva TM4C123G with the HMC5883L compass, the I2C1 module was initialized. The following steps were taken:
1. Enable the I2C1 and GPIOA peripherals.
2. Configure the appropriate GPIO pins (PA6 and PA7) for I2C SCL and SDA functions, respectively.
3. Initialize the I2C1 master module with the system clock and set it to standard mode (100kbps).
## Compass Configuration
The HMC5883L compass requires configuration to accurately read magnetic fields. The configuration process involves writing to various internal registers:
1. Configuration Register A: Set to average 8 samples per measurement and a data output rate of 15 Hz.
2. Configuration Register B: Gain configuration is set to adjust the range of the compass.
3. Mode Register: Set the compass to continuous-measurement mode.
## Reading Compass Data
The function `ReadCompass` was implemented to:
1. Set the slave address for the compass and indicate a write operation.
2. Write to the data register to specify the desired compass data register.
3. Perform a burst read operation to obtain the raw data from the compass.
4. Convert the raw data into X, Y, and Z axis measurements.
## Main Function
The main function orchestrates the following:
1. System clocks and UART are configured for debugging output.
2. I2C and compass modules are initialized.
3. In a loop, the compass data is read and adjusted for any hard-iron offset.
4. The heading is calculated using an inverse tangent operation and normalized to a 0-359 degree range.
5. The heading and raw compass data are printed to the UART.
## Calibration
Calibration values (offsets) were determined empirically to adjust for any magnetic distortions present in the environment where the compass is operated.

This document provides a concise but comprehensive overview of the steps and configurations necessary for interfacing the HMC5883L compass with the Tiva TM4C123G microcontroller. The code snippets, along with the explanations, should serve as a guideline for similar implementations.
