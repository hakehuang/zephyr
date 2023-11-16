.. zephyr:code-sample:: dmic-i2s-codec
   :name: Digital Microphone (DMIC)
   :relevant-api: audio_dmic_interface

   Perform DMIC PDM convert to PCM and use I2S to CODEC.

Overview
********

This is a very simple application intended to show how to use the :ref:`Audio DMIC
API <audio_dmic_api>`.
fistly, Audio is converted to samples in the DMIC module.
Then, the data is placed into the I2S buffer directly without DMA or CPU intervention.
Then, it is read from the I2S buffer and sent to the CODEC.
finally, the audio data will be output to Lineout of CODEC.

uart needs to be direct to flexcomm12 thourhg arduio uart,
as flexcomm0 is used for PCM transfer from DMIC.

Requirements
************

The device to be used by the sample is specified by defining a devicetree node
label named ``dmic_dev``.
The sample has been tested on :ref:`mimxrt595_evk_cm33` (mimxrt595_evk_cm33)

Building and Running
********************

The code can be found in :zephyr_file:`samples/drivers/audio/dmic`.

To build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/audio/dmic-i2s-codec
   :board: mimrt595_evk_cm33
