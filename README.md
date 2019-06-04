# ROS driver for Romeo BLE Quad

This is a ROS node for the Romeo BLE Quad and corresponding code for the STM32 on the Romeo BLE Quad.
It enables controlling 4 motors and read all encoder data via TTL serial.

* [Romeo BLE Quad](https://www.dfrobot.com/product-1563.html)

## Parameters:

* **device** (string) -- the path to the serial port.
* **baud** (int) -- baudrate. Default 9600.

## Subscribers
* **command** -- a Int32MultiArray containing exactly 4 values.

## Publishers
None.

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
