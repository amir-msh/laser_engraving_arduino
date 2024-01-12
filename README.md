# Laser Engraving Machine

Welcome to the Laser Engraving Machine repository! This project combines hardware and software to create a versatile laser engraving device.

## Overview

The laser engraving machine is built using an Arduino Uno, two precise stepper motors, a laser, Bluetooth communication, and a relay for laser control. The device utilizes two mirrors attached to the stepper motors to guide the laser beam, allowing for precise engraving on various materials.

## Features

- Bluetooth communication for easy control via [a custom Flutter app (Android is currently supported)](https://github.com/amir-msh/laser_engraving_client).
- Stepper motor-driven mirrors enable precise laser positioning.
- Suitable for engraving on wood, plastic, and non-reflective clothing materials.

## Usage

1. Connect the Arduino Uno and power up the laser engraving machine.
2. Use the custom Flutter app on your Android device to communicate with the machine.
3. Calibrate stepper motors using the provided limit switches and in-code variables.
4. Engrave images on compatible materials.

## Limitations

- Laser power is not sufficient for engraving on metallic materials.
- Works best on materials without reflective surfaces and light colors.

## License

This project is licensed under the [MIT License](LICENSE).

Happy engraving! 😃
