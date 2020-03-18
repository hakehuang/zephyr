/* test audio play back*/

/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#include <ztest.h>
#include <kernel_internal.h>


/**
 *
 * @brief Regression test's entry point
 *
 *
 * @return N/A
 */

void test_sin_wav(void)
{

}


void test_main(void)
{
	ztest_test_suite(common_test, ztest_unit_test(test_sin_wav));

	ztest_run_test_suite(common_test);
}
