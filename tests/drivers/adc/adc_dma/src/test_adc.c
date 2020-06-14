/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @addtogroup test_adc_basic_operations
 * @{
 * @defgroup t_adc_basic_basic_operations test_adc_sample
 * @brief TestPurpose: verify ADC driver handles different sampling scenarios
 * @}
 */

#include <drivers/adc.h>
#include <drivers/counter.h>
#include <zephyr.h>
#include <ztest.h>


#define ADC_DEVICE_NAME		DT_LABEL(DT_INST(0, nxp_kinetis_adc16))
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	26
#define HW_TRIGGER_INTERVAL (5000U)
/* for DMA HW trigger interval need large than HW trigger interval*/
#define SAMPLE_INTERVAL_US  (10000U)     

#define BUFFER_SIZE  256
static ZTEST_BMEM s32_t m_sample_buffer[BUFFER_SIZE];

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};
#if defined(ADC_2ND_CHANNEL_ID)
static const struct adc_channel_cfg m_2nd_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_2ND_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_2ND_CHANNEL_INPUT,
#endif
};
#endif /* defined(ADC_2ND_CHANNEL_ID) */

struct device *get_adc_device(void)
{
	return device_get_binding(ADC_DEVICE_NAME);
}

struct device *get_count_device(void)
{
	char * dev_name = DT_LABEL(DT_NODELABEL(pit0));
	return device_get_binding(dev_name);
}

static void init_pit(void)
{
	char * dev_name = DT_LABEL(DT_NODELABEL(pit0));
	int err;
	struct device *dev;
	struct counter_top_cfg top_cfg = {
		.callback = NULL,
		.user_data = NULL,
		.flags = 0
	};
	
	dev = device_get_binding(dev_name);
	zassert_not_null(dev, "Unable to get counter device %s",
				 dev_name);
	
	counter_start(dev);
	/*pit interval in 5ms sample 256 time, so the total convert rate is 51.2k*/
	top_cfg.ticks = counter_us_to_ticks(dev, 5000);
	err = counter_set_top_value(dev, &top_cfg);
	zassert_equal(0, err, "%s: Counter failed to set top value (err: %d)",
			dev_name, err);
}

static void stop_pit()
{
	char * dev_name = DT_LABEL(DT_NODELABEL(pit0));
	struct device *dev;

	dev = device_get_binding(dev_name);
	zassert_not_null(dev, "Unable to get counter device %s",
				 dev_name);
	counter_stop(dev);
}

static struct device *init_adc(void)
{
	int ret;
	struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

	zassert_not_null(adc_dev, "Cannot get ADC device");

	ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	zassert_equal(ret, 0,
		"Setting up of the first channel failed with code %d", ret);

#if defined(ADC_2ND_CHANNEL_ID)
	ret = adc_channel_setup(adc_dev, &m_2nd_channel_cfg);
	zassert_equal(ret, 0,
		"Setting up of the second channel failed with code %d", ret);
#endif /* defined(ADC_2ND_CHANNEL_ID) */

	(void)memset(m_sample_buffer, 0, sizeof(m_sample_buffer));
	
	init_pit();

	return adc_dev;
}

static void check_samples(int expected_count)
{
	int i;

	TC_PRINT("Samples read: ");
	#if 1
	for (i = 0; i < BUFFER_SIZE; i++) {
		s16_t sample_value = m_sample_buffer[i];

		TC_PRINT("0x%04x ", sample_value);
		if (i && i%10 == 0)
		{
			TC_PRINT("\n");
		}
	}
	#endif
	TC_PRINT("%d sampled\n", BUFFER_SIZE);
}

/*
 * test_adc_sample_one_channel
 */
static int test_task_one_channel(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return TC_FAIL;
	}

	ret = adc_read(adc_dev, &sequence);
	zassert_equal(ret, 0, "adc_read() failed with code %d", ret);

	check_samples(1);

	return TC_PASS;
}

void test_adc_sample_one_channel(void)
{
	zassert_true(test_task_one_channel() == TC_PASS, NULL);
}

/*
 * test_adc_sample_two_channels
 */
#if defined(ADC_2ND_CHANNEL_ID)
static int test_task_two_channels(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID) |
			       BIT(ADC_2ND_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return TC_FAIL;
	}

	ret = adc_read(adc_dev, &sequence);
	zassert_equal(ret, 0, "adc_read() failed with code %d", ret);

	check_samples(2);

	return TC_PASS;
}
#endif /* defined(ADC_2ND_CHANNEL_ID) */

void test_adc_sample_two_channels(void)
{
#if defined(ADC_2ND_CHANNEL_ID)
	zassert_true(test_task_two_channels() == TC_PASS, NULL);
#else
	ztest_test_skip();
#endif /* defined(ADC_2ND_CHANNEL_ID) */
}

/*
 * test_adc_asynchronous_call
 */
#if defined(CONFIG_ADC_ASYNC)
struct k_poll_signal async_sig;

static int test_task_asynchronous_call(void)
{
	int ret;
	const struct adc_sequence_options options = {
		.extra_samplings = 4,
		/* Start consecutive samplings as fast as possible. */
		.interval_us     = HW_TRIGGER_INTERVAL,
	};
	const struct adc_sequence sequence = {
		.options     = &options,
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};
	struct k_poll_event  async_evt =
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
					 K_POLL_MODE_NOTIFY_ONLY,
					 &async_sig);
	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return TC_FAIL;
	}

	ret = adc_read_async(adc_dev, &sequence, &async_sig);
	zassert_equal(ret, 0, "adc_read_async() failed with code %d", ret);

	ret = k_poll(&async_evt, 1, K_MSEC(1000));
	zassert_equal(ret, 0, "k_poll failed with error %d", ret);

	check_samples(1 + options.extra_samplings);

	return TC_PASS;
}
#endif /* defined(CONFIG_ADC_ASYNC) */

void test_adc_asynchronous_call(void)
{
#if defined(CONFIG_ADC_ASYNC)
	zassert_true(test_task_asynchronous_call() == TC_PASS, NULL);
#else
	ztest_test_skip();
#endif /* defined(CONFIG_ADC_ASYNC) */
}

/*
 * test_adc_sample_with_interval
 */
static enum adc_action sample_with_interval_callback(
				struct device *dev,
				const struct adc_sequence *sequence,
				u16_t sampling_index)
{
	TC_PRINT("%s: sampling %d\n", __func__, sampling_index);
	return ADC_ACTION_CONTINUE;
}

static int test_task_with_interval(void)
{
	int ret;
	const struct adc_sequence_options options = {
		.interval_us     = 100 * 1000UL,
		.callback        = sample_with_interval_callback,
		.extra_samplings = 4,
	};
	const struct adc_sequence sequence = {
		.options     = &options,
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return TC_FAIL;
	}

	ret = adc_read(adc_dev, &sequence);
	zassert_equal(ret, 0, "adc_read() failed with code %d", ret);

	check_samples(1 + options.extra_samplings);

	return TC_PASS;
}

void test_adc_sample_with_interval(void)
{
	zassert_true(test_task_with_interval() == TC_PASS, NULL);
}

/*
 * test_adc_repeated_samplings
 */
static u8_t m_samplings_done;
static enum adc_action repeated_samplings_callback(
				struct device *dev,
				const struct adc_sequence *sequence,
				u16_t sampling_index)
{
	++m_samplings_done;
	TC_PRINT("%s: done %d\n", __func__, m_samplings_done);
	if (m_samplings_done == 1U) {
		#if defined(ADC_2ND_CHANNEL_ID)
			check_samples(2);
		#else
			check_samples(1);
		#endif /* defined(ADC_2ND_CHANNEL_ID) */

		/* After first sampling continue normally. */
		return ADC_ACTION_CONTINUE;
	} else {
		#if defined(ADC_2ND_CHANNEL_ID)
			check_samples(4);
		#else
			check_samples(2);
		#endif /* defined(ADC_2ND_CHANNEL_ID) */

		/*
		 * The second sampling is repeated 9 times (the samples are
		 * written in the same place), then the sequence is finished
		 * prematurely.
		 */
		if (m_samplings_done < 10) {
			return ADC_ACTION_REPEAT;
		} else {
			return ADC_ACTION_FINISH;
		}
	}
}

static int test_task_repeated_samplings(void)
{
	int ret;
	const struct adc_sequence_options options = {
		.callback        = repeated_samplings_callback,
		/*
		 * This specifies that 3 samplings are planned. However,
		 * the callback function above is constructed in such way
		 * that the first sampling is done normally, the second one
		 * is repeated 9 times, and then the sequence is finished.
		 * Hence, the third sampling will not take place.
		 */
		.extra_samplings = 2,
		/* Start consecutive samplings as fast as possible. */
		/* but the interval shall be larger than the HW trigger*/
		.interval_us     = SAMPLE_INTERVAL_US,
	};
	const struct adc_sequence sequence = {
		.options     = &options,
#if defined(ADC_2ND_CHANNEL_ID)
		.channels    = BIT(ADC_1ST_CHANNEL_ID) |
			       BIT(ADC_2ND_CHANNEL_ID),
#else
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
#endif /* defined(ADC_2ND_CHANNEL_ID) */
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return TC_FAIL;
	}

	ret = adc_read(adc_dev, &sequence);
	zassert_equal(ret, 0, "adc_read() failed with code %d", ret);

	return TC_PASS;
}

void test_adc_repeated_samplings(void)
{
	zassert_true(test_task_repeated_samplings() == TC_PASS, NULL);
}

/*
 * test_adc_invalid_request
 */
static int test_task_invalid_request(void)
{
	int ret;
	struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = 0, /* intentionally invalid value */
	};

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return TC_FAIL;
	}

	ret = adc_read(adc_dev, &sequence);
	zassert_not_equal(ret, 0, "adc_read() unexpectedly succeeded");

#if defined(CONFIG_ADC_ASYNC)
	ret = adc_read_async(adc_dev, &sequence, &async_sig);
	zassert_not_equal(ret, 0, "adc_read_async() unexpectedly succeeded");
#endif

	/*
	 * Make the sequence parameters valid, now the request should succeed.
	 */
	sequence.resolution = ADC_RESOLUTION;

	ret = adc_read(adc_dev, &sequence);
	zassert_equal(ret, 0, "adc_read() failed with code %d", ret);

	check_samples(1);

	return TC_PASS;
}

void test_adc_invalid_request(void)
{
	zassert_true(test_task_invalid_request() == TC_PASS, NULL);
}
