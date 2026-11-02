STM32F030 DEMO BOARD (no HSE crystal)
####################################

Overview
********

Variant of the :zephyr:board:`stm32f030_demo` board for units that are
populated **without** the 8 MHz HSE crystal.

The stock devicetree enables ``clk_hse``, and the STM32 clock driver then waits
forever in ``LL_RCC_HSE_IsReady()`` inside ``set_up_fixed_clock_sources()``.
The symptom is a board that looks dead: nothing on the console at all.

This variant disables ``clk_hse`` and feeds the PLL from HSI instead. With HSI
at 8 MHz the PLL input is HSI/2 = 4 MHz, so ``mul = 12`` yields the same 48 MHz
SYSCLK as the crystal build.

Hardware
********

The rest of the board is identical to :zephyr:board:`stm32f030_demo`: same SoC
(STM32F030F4, 16 KB flash / 4 KB RAM), same UART pins (USART1 PA9/PA10 at
115200), same J-Link / STM32 flash / OpenOCD runners.

Programming
***********

.. code-block:: console

   west build -b stm32f030_demo_no_hse samples/drivers/adc/adc_sequence
   west flash --runner jlink --dev-id <J-Link serial>

Several boards can be attached at once; ``--dev-id`` selects the probe.
