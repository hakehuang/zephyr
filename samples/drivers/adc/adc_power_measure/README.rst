.. zephyr:code-sample:: adc_power_measure
   :name: Analog-to-Digital Converter (ADC) power measurement sample
   :relevant-api: adc_interface

   Read analog inputs from ADC channels, using a sequence.

Overview
********

This sample is used together with a general power measurement harness to measure
power.  At startup it configures every channel described in the devicetree as a child
node of the ADC referenced by the ``adc0`` alias, then reads all of them as a single
ADC sequence and prints each raw sample next to its millivolt equivalent.

The sample is interactive: one measurement is taken every time a character is received
on the console, which makes it easy to drive from a host script or to step through by
hand while probing a board.

Usage
*****

Console interaction
===================

The output is line oriented and meant to be consumed by a script that computes
power from the measured voltages.  At boot the sample announces the format of
the data it is going to send:

.. code-block:: console

   #CFG,ch=8,res=12,ref_mv=1250,seq=5,avg=4

.. list-table::
   :header-rows: 1

   * - Field
     - Meaning
   * - ``ch``
     - number of channels read per measurement
   * - ``res``
     - ADC resolution in bits, ``ch`` values are raw codes
   * - ``ref_mv``
     - reference voltage in millivolts
   * - ``seq``
     - samples read per channel in one measurement
   * - ``avg``
     - samples entering the reported mean, that is ``seq`` minus the discarded
       first sample (see `Measurement quality`_)

From then on:

* Send **any character** to take one measurement.
* Send ``\r`` (carriage return) to print the channel configuration instead of a
  measurement.

A measurement is one line: a timestamp followed by the mean raw value of every
channel, in the channel order of the devicetree::

   D,<uptime_ms>,<ch0>,<ch1>,...,<chN>

``uptime_ms`` is :c:func:`k_uptime_get` at the moment the sequence completed, so
the sampling instants do not depend on console latency.  The values are raw codes
averaged over the ``avg`` samples of each channel, with the first one discarded;
convert with ``mV = raw * ref_mv / (1 << res)``.

A real capture on stm32f030_demo, with one character sent per second:

.. code-block:: console

   #CFG,ch=8,res=12,ref_mv=1250,seq=5,avg=4
   D,4337,1227,1882,2061,2084,1594,2023,2222,1997
   D,5342,1843,2037,1935,2157,1603,1972,2162,1998
   D,6357,1949,2116,1983,2167,1599,2037,2201,2015

A failed read is reported as ``#ERR,<errno>`` instead of a ``D`` line:

.. code-block:: console

   #ERR,-5

.. note::

   Lines starting with ``#`` are status/comment lines: ``#CFG`` is the one-shot
   header and ``#ERR`` reports a failed read.  Data lines always start with
   ``D,``, which makes the stream trivial to filter.

A minimal consumer looks like this:

.. code-block:: python

   for line in serial_stream:
       line = line.strip()
       if line.startswith("#CFG,"):
           cfg = dict(kv.split("=") for kv in line[5:].split(","))
       elif line.startswith("D,"):
           fields = line.split(",")
           t_ms = int(fields[1])
           raw = [int(v) for v in fields[2:]]
           mv = [r * int(cfg["ref_mv"]) // (1 << int(cfg["res"])) for r in raw]

The configuration output (triggered by ``\r``) looks like this:

.. code-block:: console

   ==== start of adc features ====
   CHANNEL_COUNT: 4
   Resolution: 12
           channel_id 0 features:
            - is single mode
            - verf is 3300 mv
           channel_id 3 features:
            - is single mode
            - verf is 3300 mv
           channel_id 4 features:
            - is single mode
            - verf is 3300 mv
           channel_id 7 features:
            - is single mode
            - verf is 3300 mv
   ==== end of adc features ====

Shell mode
==========

Setting :kconfig:option:`CONFIG_SHELL_ADC_PS` replaces the ad-hoc console parsing with
shell commands; the main loop then only sleeps and the measurements are triggered
explicitly:

.. code-block:: console

   uart:~$ adc read      # take one measurement
   uart:~$ adc status    # print the channel configuration

Both commands use the same code path as the plain-console variant, so the output
format is identical.

Kconfig options
===============

.. list-table::
   :header-rows: 1

   * - Option
     - Default
     - Description
   * - :kconfig:option:`CONFIG_SEQUENCE_SAMPLES`
     - 5
     - Number of samples taken per channel in one measurement.
   * - :kconfig:option:`CONFIG_SEQUENCE_DISCARD_FIRST`
     - y
     - Drop the first sample of every channel before averaging, because it is
       taken while the sample-and-hold capacitor is still settling.
   * - :kconfig:option:`CONFIG_SEQUENCE_RESOLUTION`
     - 12
     - Resolution requested for the sequence, in bits.
   * - :kconfig:option:`CONFIG_SEQUENCE_32BITS_REGISTERS`
     - n
     - Set when the ADC returns 32-bit results, so the sample allocates ``uint32_t``
       samples instead of ``uint16_t``.  See `Data width`_.
   * - :kconfig:option:`CONFIG_SHELL_ADC_PS`
     - n
     - Expose ``adc read`` / ``adc status`` shell commands and drop the console
       character parsing.

Building and Running
********************

Devicetree requirements
=======================

Make sure that the ADC is enabled (``status = "okay";``) and has each channel as a
child node, with your desired settings like gain, reference, or acquisition time and
oversampling setting (if used). It is also needed to provide an alias ``adc0`` for the
desired adc. See :zephyr_file:`boards/frdm_mcxc444.overlay
<samples/drivers/adc/adc_power_measure/boards/frdm_mcxc444.overlay>` for an example of
such setup.

Supported platforms
===================

See overlay files in the boards folder.  A board needs its own overlay whenever it does
not already provide an ``adc0`` alias with the channels to be measured.

Building and Running for NXP frdm_mcxc444
=========================================

The sample can be built and executed for the
:zephyr:board:`frdm_mcxc444` as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/adc/adc_power_measure
   :board: frdm_mcxc444/mcxc444
   :goals: build flash
   :compact:

To build for another board, change "frdm_mcxc444/mcxc444" above to that board's name
and provide a corresponding devicetree overlay.

Building and Running for STM32F030 DEMO
=======================================

:zephyr:board:`stm32f030_demo` ships both an overlay and a board configuration file,
so no additional arguments are needed:

.. code-block:: console

   west build -p always -b stm32f030_demo samples/drivers/adc/adc_power_measure
   west flash --runner jlink

.. note::

   ``--runner jlink`` has to be selected explicitly.  The board enables the
   ``openocd``, ``stm32flash`` and ``jlink`` runners, and ``openocd`` is the default.

The board configuration uses 12-bit resolution, 5 samples per channel and a 16-bit
sample buffer.  Expect about 93% of the 16 KiB flash and 60% of the 4 KiB RAM to be
used, so verify the size report after adding any feature:

.. code-block:: console

   Memory region         Used Size  Region Size  %age Used
              FLASH:       15296 B        16 KB     93.36%
                RAM:        2472 B         4 KB     60.35%

Console
-------

The console is USART1 (PA9 TX / PA10 RX) at **115200 8N1**.  With the on-board J-Link
it enumerates as a CDC-ACM device, for example::

   /dev/serial/by-id/usb-SEGGER_J-Link_J-Link_V9.6_Plus_000069610657-if00  ->  ttyACM0

Two details matter when capturing the output:

1. Start reading *before* resetting the board.  The UART has no buffering, so opening
   the port after a reset returns nothing::

      stty -F $SER 115200 raw -echo
      timeout 20 cat $SER > capture.txt &     # start capture first
      JLinkExe -CommanderScript reset.jlink   # then reset the board

   with ``reset.jlink`` containing ``si SWD``, ``speed 4000``, ``device STM32F030F4``,
   ``connect``, ``r``, ``g``, ``q``.

2. A measurement only happens when a character is sent, so a capture that shows only
   the ``#CFG`` header is expected until something is written to the port::

      printf 'x' > $SER

Expected output on this board (one character sent per line):

.. code-block:: console

   #CFG,ch=8,res=12,ref_mv=1250,seq=5,avg=4
   D,4337,1227,1882,2061,2084,1594,2023,2222,1997
   D,5342,1843,2037,1935,2157,1603,1972,2162,1998

Hardware notes
--------------

**Acquisition time matters for multi-channel sequences.** The overlay sets
``zephyr,acquisition-time`` to 240 ADC cycles on every channel.  With the default
(shortest) acquisition time and an ADC clock of PCLK/4 = 12 MHz, a conversion finishes
before the end-of-conversion interrupt can read the data register.  The resulting
overrun aborts the sequence after the first channel, so :c:func:`adc_read` blocks
forever waiting for the remaining samples.  Keep the acquisition time long enough when
sampling several channels; alternatively, enable DMA (``CONFIG_ADC_STM32_DMA``) on
boards whose devicetree provides a DMA channel for the ADC.

Two related constraints on STM32F0 parts:

* ``zephyr,acquisition-time`` must match a value from the ``sampling-times`` table of
  the SoC devicetree (``<2 8 14 29 42 56 72 240>``) exactly, otherwise channel setup
  fails with ``-EINVAL``.
* ``st,adc-prescaler`` only accepts ``2`` or ``4`` (PCLK/2 or PCLK/4); any other value
  fails to build.

Data width
----------

``CONFIG_SEQUENCE_32BITS_REGISTERS`` must match the sample size the ADC driver returns.
On STM32F0 the driver stores ``uint16_t`` samples, so the option has to stay disabled.
Enabling it makes the sample declare a ``uint32_t`` buffer while the driver keeps
writing 16-bit values; the sample then reads interleaved garbage (for example
``149621369`` instead of ``~600``) without any error being reported.

Measurement quality
-------------------

The sample discards the first reading of every channel before averaging
(:kconfig:option:`CONFIG_SEQUENCE_DISCARD_FIRST`): it is taken while the
sample-and-hold capacitor still holds charge from the previous channel, so it is
off by a large fraction of the measured value.  On a floating input the five raw
codes of one channel can look like this:

.. code-block:: console

   639 1310 1678 1937 2089

The first sample is about 980 codes below the settled value, and the mean of the
remaining four is still 200 codes higher than the mean of all five.  Discarding
the first sample removes that bias.

.. note::

   With a high impedance source the later samples are still creeping towards the
   final value as well (see the sequence above).  Drive the input from a low
   impedance divider, raise :kconfig:option:`CONFIG_SEQUENCE_SAMPLES` and check
   that the last samples agree, before trusting the absolute value.

.. note::

   If the console shows no banner at all, the likely cause is a channel configuration
   error: the sample returns silently from ``main()`` when ``adc_channel_setup()``
   fails.  Uncommenting the ``printf()`` above that ``return 0`` makes the failing
   channel and errno visible.
