/*
 * Copyright NDP Developer 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_ndp.h>

static struct net_if *test_iface;

/**
 * @brief Test callback function
 */
static enum net_verdict test_callback(struct net_pkt *pkt)
{
    ARG_UNUSED(pkt);
    return NET_OK;
}

/**
 * @brief Test NDP callback registration
 */
ZTEST(ndp_test, test_ndp_register_callback)
{
    int ret;
    
    zassert_not_null(test_iface, "Network interface required");
    
    ret = net_ndp_register_callback(test_iface, test_callback);
    zassert_true(ret >= 0, "NDP registration should succeed");
    
    /* Clean up */
    ret = net_ndp_unregister_callback(test_iface, test_callback);
    zassert_true(ret >= 0, "NDP unregistration should succeed");
}

/**
 * @brief Test NDP callback with null parameters
 */
ZTEST(ndp_test, test_ndp_null_parameters)
{
    int ret;
    
    /* Test null interface */
    ret = net_ndp_register_callback(NULL, test_callback);
    zassert_true(ret < 0, "Should fail with null interface");
    
    /* Test null callback */
    ret = net_ndp_register_callback(test_iface, NULL);
    zassert_true(ret < 0, "Should fail with null callback");
}

/**
 * @brief Setup function for tests
 */
static void *ndp_setup(void)
{
    test_iface = net_if_get_default();
    return NULL;
}

ZTEST_SUITE(ndp_test, NULL, ndp_setup, NULL, NULL, NULL);