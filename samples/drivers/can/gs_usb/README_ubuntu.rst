# CAN FD in Ubuntu: User Guide

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Basic Configuration](#basic-configuration)
- [Enabling CAN FD](#enabling-can-fd)
- [Working with Multiple Interfaces](#working-with-multiple-interfaces)
- [Usage Examples](#usage-examples)
- [Persistent Configuration](#persistent-configuration)
- [Troubleshooting](#troubleshooting)
- [Additional Resources](#additional-resources)

## Introduction

CAN FD (Controller Area Network with Flexible Data-Rate) is an extension of the traditional CAN protocol that allows for higher data rates and larger payloads. This guide covers how to set up and use CAN FD interfaces in Ubuntu Linux.

## Prerequisites

Ubuntu 18.04 or newer is recommended for best compatibility with CAN FD features.

## Hardware Requirements

- A CAN FD compatible adapter (USB, PCI, PCIe, serial, etc.)
- Not all CAN adapters support CAN FD - verify your hardware specifications
- Common supported adapters include:
  - PEAK-System PCAN-USB FD
  - Kvaser USB interfaces with FD support
  - IXXAT USB-to-CAN FD
  - ESD CAN FD interfaces

## Installation

Install the required packages:

```bash
sudo apt update
sudo apt install can-utils linux-modules-extra-$(uname -r)

```

Load the necessary kernel modules:

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe gs_usb
```

### Basic Configuration

1. Setting Up a CAN Interface
Identify your interface (assuming a standard interface like can0):

```bash
ip link | grep can

```

2. Configure standard CAN parameters:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

3. Verify the configuration:

```bash
ip -details link show can0
```

### Enabling CAN FD

1. Configure the interface with CAN FD parameters:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on loopback on
sudo ip link set up can0
```

2. Verify CAN FD is enabled:

```bash
ip -details link show can0
```

### Working with Multiple Interfaces

Configure multiple CAN FD interfaces with compatible settings:


```bash
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on loopback on
sudo ip link set up can0
```


## Usage Examples

### Monitoring CAN Traffic

```bash
candump can0
```

### Sending Standard CAN Frames


```bash
cansend can0 123#DEADBEEF
```

### Sending CAN FD Frames

Note the double hash (##) for CAN FD frames:

```bash
cansend can0 123##DEADBEEFDEADBEEFDEADBEEF
```

### Generate Traffic

```bash
cangen can0 -g 10 -I 123 -L 64 -D i -f
```

Options explained:

```
-g 10: Generate a frame every 10ms
-I 123: Use ID 123
-L 64: 64 byte length (CAN FD)
-D i: Incrementing data pattern
-f: Use CAN FD frames
```

### Persistent Configuration

Create a configuration file:

```bash
sudo nano /etc/network/interfaces.d/can
```

Add the following content:

```
# CAN FD configuration
auto can0
iface can0 inet manual
    pre-up ip link set $IFACE type can bitrate 500000 dbitrate 2000000 fd on
    up ip link set $IFACE up
    down ip link set $IFACE down
```

Alternatively, use systemd:

```bash
sudo nano /etc/systemd/network/80-can.network

```

add below

```
[Match]
Name=can0

[CAN]
BitRate=500000
DataBitRate=2000000
FDMode=yes

[Link]
RequiredForOnline=no

```

Enable and start the service:

```
sudo systemctl enable systemd-networkd
sudo systemctl restart systemd-networkd
```

## Diagnostic Commands

Check for bus errors:

```bash
ip -details -statistics link show can0
```

## Additional Resources

- [SocketCAN Documentation](https://docs.kernel.org/networking/can.html)
- [can-utils GitHub Repository](https://github.com/linux-can/can-utils)
- [Linux Kernel CAN Documentation](https://www.kernel.org/doc/Documentation/networking/can.txt)