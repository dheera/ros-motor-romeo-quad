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
