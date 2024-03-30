# Wall Follower with Ultrasonic Sensor and PID Controller

This project is a wall follower implementation that uses a single ultrasonic sensor and a PID (Proportional-Integral-Derivative) controller to follow a wall. The project is written in C and is designed to run on an AVR microcontroller.

## Hardware Requirements

- AVR microcontroller board (e.g., Arduino Uno)
- Ultrasonic sensor (e.g., HC-SR04)
- Two DC motors with motor drivers

## Software Components

The project includes the following key components:

1. **Ultrasonic Sensor**: The ultrasonic sensor is used to measure the distance between the robot and the wall. The `trigPin` and `echoPin` are defined to interface with the sensor.

2. **Motor Control**: The project includes custom functions (`makeOutput`, `checkInput`, and `customAnalogWrite`) to control the motors using PWM (Pulse Width Modulation) signals.

3. **Serial Communication**: The project initializes serial communication for debugging purposes and transmits the measured distance to the serial monitor.

4. **Timer and Interrupts**: The project utilizes Timer0 for a millis-like functionality, and Timer1 for measuring the pulse duration from the ultrasonic sensor. Interrupts are used to handle the timing events.

5. **PID Controller**: The PID controller is implemented to adjust the motor speeds based on the error between the measured distance and the desired setpoint (`setpoint`). The PID gains (`kp`, `ki`, and `kd`) can be adjusted to tune the controller's performance.

## Usage

1. Connect the ultrasonic sensor to the microcontroller according to the specified pin assignments (`trigPin` and `echoPin`).
2. Connect the motors to the appropriate pins, considering the motor driver configuration.
3. Upload the code to the microcontroller.
4. Open the serial monitor to observe the distance measurements.
5. The robot should start following the wall, adjusting its motor speeds based on the PID controller's output.

## Customization

You can customize the project by modifying the following parameters:

- `DOT_LENGTH`: Adjust the dot length for the motor control signals.
- `TIMEOUT_US`: Adjust the timeout value for the ultrasonic sensor measurements.
- `kp`, `ki`, and `kd`: Tune the PID gains for optimal performance based on your robot's characteristics.
- `setpoint`: Adjust the desired distance from the wall.

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE).
