.. _ndp-sample:

NDP Sample Application
======================

Overview
--------

This sample demonstrates the Network Data Plane (NDP) callback
system in Zephyr.
It registers a callback that intercepts network
packets before they enter the standard protocol stack, allowing
for high-performance packet processing, filtering, or forwarding.

The NDP callback provides:
- Packet interception and preprocessing
- Performance metrics (throughput, latency, packet rate)
- A 60-second built-in performance test with periodic reporting

Two modes are supported:
- **Mixed mode** (default): NDP preprocesses packets, then passes
  them to the native TCP/IP stack for normal processing.
- **Pure NDP mode**: NDP processes packets entirely, bypassing the
  native stack (enabled via CONFIG_NDP_PURE=y).

Prerequisites
-------------

Before building, ensure you have:

- Zephyr RTOS development environment
- Zephyr SDK 0.17.1+ (works with modified version check) or SDK 1.0.0
- West build tool installed
- QEMU (for emulation targets)

To verify your environment:

.. code-block:: console

   west --version
   python3 --version

Building for native_sim
-----------------------

The native_sim board builds a POSIX executable that runs directly
on the host and creates a virtual network interface.

.. code-block:: console

   west build -b native_sim -p auto samples/net/ndp

To run:

.. code-block:: console

   ./build/zephyr/zephyr.exe

Building for qemu_x86 with e1000 Networking
-------------------------------------------

The qemu_x86 board requires extra configuration for networking.
Two overlay files are provided in the sample directory:

1. **overlay-e1000.conf** - Kconfig fragment enabling e1000 driver,
   PCI Express, DHCP, network shell, and statistics.
2. **e1000.overlay** - Devicetree overlay adding the e1000 Ethernet
   device to the PCIe bus.

.. code-block:: console

   west build -b qemu_x86 -p auto samples/net/ndp ^
     -DEXTRA_CONF_FILE=overlay-e1000.conf ^
     -DDTC_OVERLAY_FILE=e1000.overlay

The build produces:
- build/zephyr/zephyr.elf - ELF binary for QEMU

Running in QEMU with Networking
-------------------------------

.. code-block:: console

   qemu-system-i386 -m 32 -cpu qemu32,+nx,+pae -machine q35 ^
     -device isa-debug-exit,iobase=0xf4,iosize=0x04 -no-reboot ^
     -machine acpi=off ^
     -netdev user,id=net0 ^
     -device e1000,netdev=net0 ^
     -nographic ^
     -kernel build/zephyr/zephyr.elf


**QEMU options explained:**
- ``-netdev user,id=net0``: User-mode network backend.
  Guest is on 10.0.2.0/24 with IP 10.0.2.15.
- ``-device e1000,netdev=net0``: Attaches e1000 NIC.
- ``-nographic``: Serial console on stdio.

On Windows:

.. code-block:: batch

   "C:\Program Files\qemu\qemu-system-i386.exe" ^
       -m 32 -cpu qemu32,+nx,+pae -machine q35 ^
       -device isa-debug-exit,iobase=0xf4,iosize=0x04 -no-reboot ^
       -machine acpi=off ^
       -netdev user,id=net0 ^
       -device e1000,netdev=net0 ^
       -chardev stdio,id=con,mux=on ^
       -serial chardev:con -mon chardev=con,mode=readline ^
       -nographic ^
       -kernel build\zephyr\zephyr.elf



Building for qemu_x86 with VirtIO Networking
--------------------------------------------

As an alternative to the e1000 emulated NIC, the NDP sample can use
the **virtio-net** paravirtualized network device, which provides
better performance through a simpler device model.

Three files are provided for VirtIO networking:

1. **overlay-virtio.conf** - Kconfig fragment enabling the virtio-net
   driver (`CONFIG_ETH_VIRTIO_NET`), PCI Express support, DHCP,
   network shell, and statistics.
2. **virtio.overlay** - Devicetree overlay adding a virtio-net device
   on the PCIe bus using the `virtio,pci` transport binding
   (vendor ID `0x1af4` = Red Hat / QEMU, device ID `0x1000`).
3. **run_qemu_virtio.bat** - QEMU launcher using `-device virtio-net-pci`.

.. code-block:: console

   west build -b qemu_x86 -p auto samples/net/ndp ^
     -DEXTRA_CONF_FILE=overlay-virtio.conf ^
     -DDTC_OVERLAY_FILE=virtio.overlay

The QEMU command uses `virtio-net-pci` instead of `e1000`:

.. code-block:: console

   qemu-system-i386 -m 32 -cpu qemu32,+nx,+pae -machine q35 ^
     -device isa-debug-exit,iobase=0xf4,iosize=0x04 -no-reboot ^
     -machine acpi=off ^
     -netdev user,id=net0 ^
     -device virtio-net-pci,netdev=net0 ^
     -nographic ^
     -kernel build/zephyr/zephyr.elf

On Windows:

.. code-block:: batch

   "C:\\Program Files\\qemu\\qemu-system-i386.exe" ^
       -m 32 -cpu qemu32,+nx,+pae -machine q35 ^
       -device isa-debug-exit,iobase=0xf4,iosize=0x04 -no-reboot ^
       -machine acpi=off ^
       -netdev user,id=net0 ^
       -device virtio-net-pci,netdev=net0 ^
       -chardev stdio,id=con,mux=on ^
       -serial chardev:con -mon chardev=con,mode=readline ^
       -nographic ^
       -kernel build\\zephyr\\zephyr.elf

.. note::

   The virtio-net driver uses PCI transport. The `virtio.overlay`
   disables the e1000 device (`status = "disabled"`) to avoid
   conflicting with the virtio device.


Sending Traffic to the NDP Sample
----------------------------------

Once QEMU is running, the NDP application will wait for incoming
network traffic. After receiving a packet, the NDP callback is
triggered and performance test results are reported every 5 seconds
for a total duration of 60 seconds.

The QEMU user-mode network gives the guest IP 10.0.2.15.
The host can reach the guest at 10.0.2.15 by default.

.. code-block:: console

   # Ping the guest to verify connectivity
   ping 10.0.2.15

   # Send UDP packets using netcat (Linux)
   echo "test" | nc -u 10.0.2.15 4242

   # Or use a flood ping for more traffic
   ping -f 10.0.2.15

   # Or send a burst of UDP packets (Linux)
   for i in $(seq 1 100); do
     dd if=/dev/zero bs=64 count=1 2>/dev/null | nc -u 10.0.2.15 4242
   done

On Windows, use PowerShell:

.. code-block:: powershell

   # Test reachability
   ping 10.0.2.15 -n 1

   # Send UDP packets
   $port = 4242
   $addr = [System.Net.IPAddress]::Parse("10.0.2.15")
   $ep = New-Object System.Net.IPEndPoint $addr, $port
   $udp = New-Object System.Net.Sockets.UdpClient
   $bytes = [byte[]]::new(64)
   $udp.Send($bytes, 64, $ep) | Out-Null
   $udp.Close()

   # Send in a loop
   1..100 | ForEach-Object {
       $udp = New-Object System.Net.Sockets.UdpClient
       $bytes = [byte[]]::new(64)
       $udp.Send($bytes, 64, $ep) | Out-Null
       $udp.Close()
   }

Expected Performance Output
---------------------------

When the NDP sample receives network traffic, it prints performance
statistics every 5 seconds:

.. code-block:: console

   [00:00:05.000] NDP Performance Report:
   Packets processed: 1250
   Bytes processed:   80000
   Throughput:        128000 bps
   Packets/sec:       250
   Min latency:       12 us
   Max latency:       87 us
   Avg latency:       34 us
   Drop rate:         0.0%

   [00:00:10.000] NDP Performance Report:
   Packets processed: 2500
   Bytes processed:   160000
   Throughput:        128000 bps
   Packets/sec:       250
   Min latency:       10 us
   Max latency:       95 us
   Avg latency:       32 us
   Drop rate:         0.0%

The test runs for 60 seconds. Press Ctrl+A then X to exit QEMU.
