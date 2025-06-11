# hex-pcw-direct-can-control-demo
The demo code to use rust/python to control hex-pcw using SocketCan directly

This demo will only control one pcw. Make sure you set the object dictionary of pcw to the correct value(6A01 should be 1, 6A02 should be 0).

[TOC]

## Common Setup
1. Makesure your system can handle can fd. Set system with correct bitrate (Default is 1M, 2M) `ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on` and `ip link set can0 up` (Replace can0 with your can interface)
2. Make sure the OD settings are correct. (This only has to be done once) For this example, 0x6A01-00 should be 1, 0x6A02-00 should be 0. For meaning of these numbers, check our documentation website.

### CANOpenLinux
In case you don't have tools to read/write OD, you can use CANOpenLinux to do so.

1. Clone the repo using `git clone https://github.com/CANopenNode/CANopenLinux.git --recursive`
2. Build the project using `make`
3. Bring up can
4. Run the canopenlinux using `./canopend can0 -i 2 -c stdio` (Replace can0 with your can interface)
5. Type `0x10 r 0x6A01 0` to check the value of 0x6A01-00
6. Type `0x10 r 0x6A02 0` to check the value of 0x6A02-00
7. Type `0x10 w 0x6A01 0 u8 1` to set the value of 0x6A01-00 to 1
8. Type `0x10 w 0x6A02 0 u8 0` to set the value of 0x6A02-00 to 0

![CANOpenLinux](./CANOpenLinux.png)

## Python

## Rust

1. Make sure everything in common setup is done.
2. Run `cargo run --release -- -c can0` (Replace can0 with your can interface)
3. Watch it rotate
