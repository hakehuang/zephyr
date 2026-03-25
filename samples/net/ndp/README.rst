NDP Sample Application
=======================

Overview
--------

This sample demonstrates the Network Data Plane (NDP) callback system in Zephyr.
The application registers a callback function that processes network packets
before they enter the standard protocol stack.

Features
--------

- Registers NDP callback for packet processing
- Demonstrates packet filtering based on size
- Shows network interface information
- Displays Ethernet statistics

Requirements
------------

- Network interface with Ethernet support
- NDP packet processing enabled in configuration

Building and Running
--------------------

1. Configure the application::

   west build -b <board> zephyr/samples/net/ndp

2. Flash the application::

   west flash

3. Monitor the output::

   minicom -D <serial_device> -b 115200

Configuration
-------------

Key configuration options:

- ``CONFIG_NDP_PACKET_PROCESSING``: Enable NDP packet processing
- ``CONFIG_NETWORKING``: Enable networking stack
- ``CONFIG_NET_L2_ETHERNET``: Enable Ethernet support

Sample Output
-------------

.. code-block:: console

   [00:00:00.000,000] <inf> ndp_sample: NDP Sample Application Started
   [00:00:00.000,000] <inf> ndp_sample: Interface: eth0
   [00:00:00.000,000] <inf> ndp_sample: Interface index: 0
   [00:00:00.000,000] <inf> ndp_sample: MAC address: 00:11:22:33:44:55
   [00:00:00.000,000] <inf> ndp_sample: NDP callback registered
   [00:00:05.000,000] <inf> ndp_sample: Stats - RX: 1024, TX: 512, Errors: 0
   [00:00:10.000,000] <inf> ndp_sample: NDP callback: packet len=64, proto=0x0800

Testing
-------

Send network traffic to the device to see the NDP callback in action.
The callback will log packet information and may drop packets based on
configured criteria.