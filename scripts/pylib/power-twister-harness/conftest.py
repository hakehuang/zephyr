# Copyright: (c)  2025, Intel Corporation
# Copyright 2025 NXP
import json
import logging

import pytest
from twister_harness import DeviceAdapter

PROBES = ('stm_powershield', 'general_powershield')


def pytest_addoption(parser):
    parser.addoption('--testdata')


@pytest.fixture
def probe_class(request, probe_path, duts):
    """Instantiate the power monitor and connect it to its measuring device.

    ``stm_powershield`` talks to the STM32 power shield whose serial port is
    passed as the ``pm_probe:<port>`` fixture, as before.

    ``general_powershield`` talks to a board running
    ``samples/drivers/adc/adc_power_measure``.  That board is the second DUT of
    the test, reserved through ``harness_config.required_devices`` (the same
    mechanism as ``tests/subsys/testsuite/multidut``), so it arrives here as
    ``duts[1]`` -- there is no separate device plumbing in this conftest anymore.
    """
    probe = None
    if request.param == 'stm_powershield':
        from stm32l562e_dk.PowerShield import PowerShield

        probe = PowerShield()
        probe.connect(probe_path)
        probe.init()

    elif request.param == 'general_powershield':
        from general_adc_platform.GeneralAdcPowerMonitor import GeneralPowerShield

        if len(duts) < 2:
            pytest.fail(
                'general_powershield needs a second device: declare it with '
                'harness_config.required_devices so twister reserves it as duts[1]'
            )
        probe = GeneralPowerShield()
        probe.connect(duts[1])
        probe.init()

    yield probe

    if probe is not None:
        probe.disconnect()


@pytest.fixture(name='probe_path', scope='session')
def fixture_probe_path(request, dut: DeviceAdapter):
    probe_port = None
    for fixture in dut.device_config.fixtures:
        if fixture.startswith('pm_probe'):
            probe_port = fixture.split(':')[1]
    return probe_port


@pytest.fixture(name='test_data', scope='session')
def fixture_test_data(request):
    # Get test data from the configuration and parse it into a dictionary
    measurements = request.config.getoption("--testdata")
    measurements = measurements.replace("'", '"')  # Ensure the data is properly formatted as JSON
    measurements_dict = json.loads(measurements)

    # Data validation
    required_keys = [
        'elements_to_trim',
        'min_peak_distance',
        'min_peak_height',
        'peak_padding',
        'measurement_duration',
        'num_of_transitions',
        'expected_rms_values',
        'tolerance_percentage',
    ]

    for key in required_keys:
        if key not in measurements_dict:
            logging.error(f"Missing required test data key: {key}")
            pytest.fail(f"Missing required test data key: {key}")

    return measurements_dict
