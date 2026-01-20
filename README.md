# EDMO IDE

This project is part of Project 3-1 (BCS3300) at Maastricht University.

This firmware is what directly controls the robot, it receives commands via a serial interface from the [EDMO-Server](https://github.com/macluxHD/EDMO-Server) project.

## Run Locally

### Prerequisites

- Git
- [Arduino CLI](https://docs.arduino.cc/arduino-cli/installation/)

### Setup Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/macluxHD/ArduinoFirmware
   ```

2. **Navigate to the project directory**
   ```bash
   cd ArduinoFirmware
   ```

3. **Install dependencies**
   ```bash
   arduino-cli lib install "Adafruit BNO08x"
   ```

4. **Compile**
   ```bash
   arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m0 .
   ```

5. **Upload**

   Change the `--port` parameter to the corresponding port that the robot is connected to on windows it may be `COM2` or `COM3` etc
   
   ```bash
   arduino-cli upload --fqbn adafruit:samd:adafruit_feather_m0 --port /dev/ttyACM0 .
   ```


