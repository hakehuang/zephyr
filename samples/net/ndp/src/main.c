/*
 * Copyright NDP Developer 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ethernet.h>

LOG_MODULE_REGISTER(ndp_sample, CONFIG_NDP_SAMPLE_LOG_LEVEL);

/**
 * @brief NDP callback function for packet processing (Mixed Mode)
 */
static enum net_verdict ndp_mixed_callback(struct net_pkt *pkt)
{
    /* Simple packet processing logic */
    LOG_INF("NDP: Processing packet in mixed mode");
    /* Custom packet processing logic here */
    return NET_CONTINUE; /* Continue with native stack */
}

/**
 * @brief NDP callback function for packet processing (NDP-Only Mode)
 */
static enum net_verdict ndp_only_callback(struct net_pkt *pkt)
{
    /* Simple packet processing logic for NDP-only mode */
    LOG_INF("NDP-Only: Processing packet");
    /* Custom packet processing logic here */
    return NET_CONTINUE; /* Continue with native stack */
}

/**
 * @brief Main application entry point
 */
int main(void)
{
    int ret;
    struct net_if *iface;

    LOG_INF("NDP Sample Application Started");

    /* Get first network interface */
    iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));
    if (iface == NULL) {
        LOG_ERR("No Ethernet interface found");
        return -1;
    }

    LOG_INF("Ethernet interface found");

    /* Register NDP callback based on configuration */
#ifdef CONFIG_NDP_ONLY_MODE
    ret = net_ndp_register_callback(iface, ndp_only_callback);
    LOG_INF("NDP-Only Mode: Custom packet processing enabled");
#else
    ret = net_ndp_register_callback(iface, ndp_mixed_callback);
    LOG_INF("Mixed Mode: NDP preprocessing + native stack");
#endif

    if (ret < 0) {
        LOG_ERR("Failed to register NDP callback: %d", ret);
        return -1;
    }

    /* Main loop */
    while (1) {
        /* Simple counter for demonstration */
        static uint32_t counter = 0;
        LOG_INF("NDP sample running for %u seconds", counter++);

        k_sleep(K_SECONDS(5));
    }

    return 0;
}