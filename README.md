# ROS driver for Romeo BLE Quad

This is a ROS node for the Romeo BLE Quad and corresponding code for the STM32 on the Romeo BLE Quad.
It enables controlling 4 motors and read all encoder data via TTL serial.

This is the only reasonably-priced 4-channel brushed motor controller I could find, and useful for small low-power indoor mecanum-drive robots. It's not very high power but it's priced right at $40. 
* [Romeo BLE Quad](https://www.dfrobot.com/product-1563.html)

Competing solutions to control 4 brushed motors include:
* Two 2x7A RoboClaws which would cost you an insane $164.
* Two dumb motor drivers (cheap, $10-$20, but no encoder counters) + 2 dual LS7366R quadrature encoder counters from SuperDroidRobots, which would cost you an obscene $104.
Nobody except DFRobot seems to understand that a robot this small shouldn't cost $100 for a stupid motor controller, and it is downright **wrong** to cost $104 to count silly encoders.

Programming the STM32 is a little tricky as if you don't get the interrupt priorities correctly it becomes difficult to (a) count encoder ticks, (b) receive data from UART, and (c) PWM drive 4 H-bridges or 8 channels all at the same time. This package solves that for you.

This package only implements open-loop PWM control. It does **not** implement closed-loop PID control of the motors. You can implement that in ROS with much more flexibility than you have on the STM32, and incorporate other (e.g. IMU) data if you do that part on the ROS stack instead of in firmware.

## Parameters:

* **device** (string) -- the path to the serial port.
* **baud** (int) -- baudrate. Default 9600.

## Subscriber
* **command** -- a Int32MultiArray containing exactly 4 values. Each value should range from -32768 to 32767 inclusive and specify the duty cycle to drive the motor.

## Publishers
* **ticks** -- (Int32MultiArray) Total encoder tick count for the 4 channels since the node was started. CAUTION: All 4 channels will reset to 0 if the node is killed and restarted. Mind this when you write any odometry-related logic.
* **ticks_diff** -- (Int16MultiArray) Differential tick count for the 4 channels.

## Services
None.

## TTL serial protocol description
Provided for reference. ROS node in repo already implements this, so you don't need to read this if you don't feel like.

### Packetized serial protocol
Packetized serial, 8 bytes per packet, in the following structure:
[1 byte: start byte] [1 byte: command] [7 bytes: args (command-dependent)] [1 byte: checksum]
* start byte: always 0xF0
* checksum: sum(payload) % 255

### Setting motor speed
PC sends the following packet to Romeo.
* command = 0x47 (set motor speed)
* args: [1 byte: channel] [2 bytes: value, signed 16-bit int, little endian] [4 bytes: dummy filler]

### Romeo->PC Direction (encoder readout)
Romeo sends the following to PC.
* command = 0x45 (report encoder diff)
* args: [1 byte: channel 0 diff, signed 8-bit int] [1 byte: channel 1 diff, signed 8-bit int] [1 byte: channel 2 diff, signed 8-bit int] [1 byte: channel 3 diff, signed 8-bit int] [4 bytes: dummy filler]
