Overview
--------

SPI loop back test

Tests
-----

drivers.spi.loopback:
    harness: ztest
    harness_config:
      fixture: spi_loopback
driver.spi.loopback.internal:
    filter: CONFIG_SPI_LOOPBACK_MODE_LOOP

Setup
------

FRDM_K64F:

need short PTD2-PTD3, e.g. J2-10 to J2-8
