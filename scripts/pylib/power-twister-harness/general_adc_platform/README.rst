.. _general_adc_platform:

General ADC power shield
========================

Power measurement platform where the measuring board runs
:zephyr:code-sample:`adc_power_measure` and reports its ADC readings over the
console.  The harness turns those readings into currents and hands them to the
shared ``scripts/pylib/power-twister-harness/test_power.py`` test, which compares
the RMS values against the expectations from the test case.

Console protocol
****************

The sample announces its configuration once at boot and then answers one
character with one measurement::

   #CFG,ch=8,res=12,ref_mv=1250,seq=5,avg=4
   D,<uptime_ms>,<ch0>,<ch1>,...,<chN>

* ``ch`` / ``res`` / ``ref_mv`` describe the raw codes (``mV = raw * ref_mv / 2**res``)
* ``seq`` is how many ADC samples were converted, ``avg`` how many of them were
  averaged into the reported value (the first one is discarded, see the sample's
  README)
* ``#ERR,<errno>`` replaces a ``D`` line when the sequence could not be read

Requirements on the twister side
********************************

The measuring board is the **second DUT** of the test, reserved through
``harness_config.required_devices`` — the same mechanism the
``tests/subsys/testsuite/multidut`` suite uses.  ``duts[0]`` is the DUT under
test, ``duts[1]`` the board that measures it::

   common:
     harness: power
     harness_config:
       fixture: pm_probe
       required_devices:
         - platform: stm32f030_demo        # the measuring board

Select this probe with ``PROBE_CLASS=general_powershield`` in the environment;
the default remains the STM32 power shield.

Configuration
*************

``probe_settings.yaml`` next to this file describes the measurement routes:

.. list-table::
   :header-rows: 1

   * - Field
     - Meaning
   * - ``name``
     - route name, used as the key in the per-route results
   * - ``shunt_resistor``
     - shunt value in ohms; current is ``V / (shunt_resistor * gain)``
   * - ``gain``
     - additional gain of the sense path (default 1.0)
   * - ``type``
     - ``single`` measures ``channels_p`` against ground, ``differential``
       measures the drop between ``channels_p`` and ``channels_n``
   * - ``calibration``
     - ``scale`` / ``offset`` applied to every channel voltage before conversion

Only routes whose channels are present in the ``D`` line are evaluated; a missing
channel is logged and skipped rather than turning into a wrong reading.

Data produced
*************

``GeneralPowerShield.get_data()`` returns one total current value (amperes) per
measurement, which is what the shared RMS logic expects.  The per-route currents
and the raw voltages stay available on the object, and the capture is written to
``power_raw_data.csv`` in the DUT build directory when the harness sets that path.
