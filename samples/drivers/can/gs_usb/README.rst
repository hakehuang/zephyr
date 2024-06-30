.. zephyr:code-sample:: gs-usb
   :name: gs_usb
   :relevant-api: gs_usb usb can

   SocketCAN firmware implementation for gs_usb Linux driver

Overview
********

The gs_usb CAN sample is a [OpenMoko Geschwister Schneider](https://linux-hardware.org/?id=usb:1d50-606f) Firmware compatible with
[gs_usb](https://github.com/torvalds/linux/blob/master/drivers/net/can/usb/gs_usb.c) Linux driver.

The application allows sending CAN-Bus frames back and forth from this Firmware to
the Linux host providing SocketCAN interface.

The source code for this sample application can be found at:
:zephyr_file:`samples/net/sockets/can`.

Requirements
************

You need a CAN-Bus enabled board like :ref:`nucleo_h745zi_q` or better two and a Linux host or
[python-can](https://python-can.readthedocs.io/en/stable/interfaces/gs_usb.html) if you prefer to use python.

Very helpful can be a bash script under [scripts/can_dev_test.sh] located in the folder of this sample.

Building and Running
********************

Build the socket CAN sample application like this:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/can/gs_usb
   :board: <board to use>
   :overlay: <overlay to use>
   :conf: <config file to use>
   :goals: build
   :compact:

Example building for the nucleo_h745zi_q_m7:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/can/gs_usb
   :host-os: unix
   :board: nucleo_h745zi_q_m7
   :overlay: nucleo_h745zi_q.overlay
   :goals: run
   :compact:
