# Copyright 2025 NXP
#
# SPDX-License-Identifier: Apache-2.0

"""Power monitor for boards that measure their own rail voltages with an ADC.

The firmware side is ``samples/drivers/adc/adc_power_measure``: it announces its
configuration once at boot and then answers one character with one measurement,

    #CFG,ch=8,res=12,ref_mv=1250,seq=5,avg=4
    D,<uptime_ms>,<ch0>,<ch1>,...,<chN>

where the values are raw ADC codes averaged over ``avg`` of ``seq`` samples.
This monitor turns that stream into currents: raw codes -> millivolts ->
volts -> amps through the shunt resistor of each configured route.

The measurement device is the second DUT reserved for the test through
``harness_config.required_devices`` (see ``tests/subsys/testsuite/multidut``),
not a hand-built device: ``duts[0]`` is the DUT under test and ``duts[1]`` the
board doing the measuring.
"""

import csv
import logging
import re
from pathlib import Path
from time import monotonic
from types import SimpleNamespace

from abstract.PowerMonitor import PowerMonitor

logger = logging.getLogger(__name__)

CFG_RE = re.compile(
    r'^#CFG,ch=(?P<ch>\d+),res=(?P<res>\d+),ref_mv=(?P<ref_mv>\d+)'
    r',seq=(?P<seq>\d+),avg=(?P<avg>\d+)'
)
DATA_RE = re.compile(r'^D,(?P<ms>\d+),(?P<raw>-?\d+(?:,-?\d+)*)$')
ERROR_RE = re.compile(r'^#ERR,(?P<errno>-?\d+)')

DEFAULT_SETTINGS = {
    'device_id': 'adc_power_monitor',
    'calibration': {'offset': 0.0, 'scale': 1.0},
    'routes': [],
}


class Route:
    """One measurement path: a shunt resistor plus the channel(s) that sense it."""

    def __init__(self, data):
        self.name = data['name']
        self.shunt = float(data['shunt_resistor'])
        self.gain = float(data.get('gain', 1.0))
        self.differential = data.get('type', 'single') == 'differential'
        channels = data['channels']
        self.channel_p = int(channels['channels_p'])
        self.channel_n = int(channels.get('channels_n', self.channel_p))

    def channels(self):
        return {self.channel_p, self.channel_n}

    def voltage_to_current(self, volts: float) -> float:
        """Ohm's law through the shunt, including the route gain."""
        if self.shunt <= 0:
            raise ValueError(f'route {self.name}: shunt_resistor must be positive')
        return volts / (self.shunt * self.gain)


def _load_settings(path=None):
    """Load the probe settings, or fall back to a single all-channel-free default.

    The file is small and hand written, so it is parsed directly instead of being
    run through a schema: a schema here only bought a second description of the
    same keys.
    """
    if path is None:
        path = Path(__file__).with_name('probe_settings.yaml')
    if not Path(path).exists():
        logger.warning('probe settings %s missing, no routes configured', path)
        return dict(DEFAULT_SETTINGS)

    import yaml  # imported lazily: the module stays importable without PyYAML

    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    settings = dict(DEFAULT_SETTINGS)
    settings.update({k: v for k, v in data.items() if v is not None})
    return settings


class GeneralPowerShield(PowerMonitor):
    """PowerMonitor that reads the adc_power_measure protocol over a serial port."""

    def __init__(self, settings_path=None):
        self.settings = _load_settings(settings_path)
        self.routes = [Route(r) for r in self.settings.get('routes', [])]
        self.calibration = self.settings.get('calibration', {})
        self.dev = None
        self.cfg = None
        self.is_initialized = False
        self.samples = []  # list[dict[int, float]]: channel -> volts, one entry per measurement
        # The shared test harness writes the raw capture to this path when the
        # DUT build directory exists, so keep the attribute it expects.
        self.power_shield_conf = SimpleNamespace(output_file=None)

    # -- PowerMonitor interface ------------------------------------------------

    def connect(self, device):
        """Read the one-shot configuration banner from the measuring device."""
        self.dev = device
        self.cfg = self._read_cfg()
        if self.cfg is None:
            raise RuntimeError(
                'no #CFG banner received from the power measurement device - '
                'is samples/drivers/adc/adc_power_measure running on it?'
            )
        logger.info('power shield configuration: %s', self.cfg)

    def init(self, device_id: str = None) -> bool:
        if device_id:
            self.settings['device_id'] = device_id
        if not self.routes:
            logger.warning('no routes configured in probe_settings.yaml')
        self.is_initialized = True
        return True

    def measure(self, time: int, freq: str = None, reset: bool = False):
        """Ask for one measurement at a time until ``time`` seconds have passed."""
        if not self.is_initialized:
            raise RuntimeError('Power monitor not initialized. Call init() first.')
        if reset:
            self.samples = []

        deadline = monotonic() + time
        while monotonic() < deadline:
            self.dev.write(b'\n')
            raw = self._read_sample()
            if raw is not None:
                self.samples.append(raw)
        logger.info('collected %d measurements', len(self.samples))
        if self.power_shield_conf.output_file:
            self.dump(self.power_shield_conf.output_file)

    def get_data(self, duration: int = 0) -> list[float]:
        """Return the total current of every measurement, in amperes.

        A flat per-sample series keeps this compatible with the RMS helpers used
        by the harness; the per-route detail stays available on the object for
        reporting.
        """
        if not self.is_initialized:
            raise RuntimeError('Power monitor not initialized. Call init() first.')

        series = self.samples[-duration:] if duration else self.samples
        totals = []
        for sample in series:
            totals.append(sum(self._sample_currents(sample).values()))
        return totals

    def disconnect(self):
        if self.dev is not None:
            self.dev.close()
            self.dev = None

    # -- helpers ---------------------------------------------------------------

    def _read_cfg(self):
        lines = self.dev.readlines_until(regex=r'^#CFG,', timeout=5, print_output=False)
        for line in lines or []:
            match = CFG_RE.match(line.strip())
            if match:
                cfg = {k: int(v) for k, v in match.groupdict().items()}
                cfg['full_scale'] = 1 << cfg['res']
                return cfg
        return None

    def _read_sample(self):
        """Triggered measurement: parse one D line into channel voltages."""
        lines = self.dev.readlines_until(regex=r'^(D,|#ERR,)', timeout=5, print_output=False)
        for line in lines or []:
            line = line.strip()
            if match := ERROR_RE.match(line):
                logger.warning('power shield reported error %s', match.group('errno'))
                return None
            if match := DATA_RE.match(line):
                raw = [int(v) for v in match.group('raw').split(',')]
                return {i: self._raw_to_volts(v) for i, v in enumerate(raw)}
        return None

    def _raw_to_volts(self, raw: int) -> float:
        if not self.cfg:
            return 0.0
        millivolts = raw * self.cfg['ref_mv'] / self.cfg['full_scale']
        volts = millivolts / 1000.0
        return volts * float(self.calibration.get('scale', 1.0)) + float(
            self.calibration.get('offset', 0.0)
        )

    def _sample_currents(self, sample: dict) -> dict:
        """Current per route for one measurement, in amperes."""
        currents = {}
        for route in self.routes:
            if not route.channels().issubset(sample):
                logger.warning('route %s: channel data missing, skipped', route.name)
                continue
            if route.differential:
                volts = abs(sample[route.channel_p] - sample[route.channel_n])
            else:
                volts = sample[route.channel_p]
            currents[route.name] = route.voltage_to_current(volts)
        return currents

    def dump(self, target=None):
        """Write the collected measurements as one CSV, for post mortem analysis."""
        if not self.samples:
            logger.warning('nothing to dump')
            return None
        path = Path(target or 'power_raw_data.csv')
        if path.is_dir():
            path = path / 'power_raw_data.csv'
        channels = sorted({c for s in self.samples for c in s})
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['sample'] + [f'ch{c}_mv' for c in channels] + ['total_a'])
            for index, sample in enumerate(self.samples):
                writer.writerow(
                    [index]
                    + [round(sample.get(c, 0.0) * 1000.0, 3) for c in channels]
                    + [round(sum(self._sample_currents(sample).values()), 6)]
                )
        logger.info('power data written to %s', path)
        return path
