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
#include <zephyr/net/net_ndp.h>

LOG_MODULE_REGISTER(ndp_sample, CONFIG_NDP_SAMPLE_LOG_LEVEL);

/**
 * @brief NDP callback function for packet processing (Mixed Mode)
 */
static enum net_verdict ndp_mixed_callback(struct net_pkt *pkt)
{
    /* Log packet information */
    LOG_INF("NDP Mixed Mode: Processing packet");
    
    /* Example: Check packet type and make decisions */
    if (net_pkt_family(pkt) == AF_INET) {
        LOG_INF("IPv4 packet");
    } else if (net_pkt_family(pkt) == AF_INET6) {
        LOG_INF("IPv6 packet");
    }
    
    /* Return CONTINUE to let packet continue to native stack */
    return NET_CONTINUE;
}

/**
 * @brief NDP callback function for packet processing (Pure NDP Mode)
 */
static enum net_verdict ndp_pure_callback(struct net_pkt *pkt)
{
    /* Log packet information */
    LOG_INF("NDP Pure Mode: Processing packet");
    
    /* Example: Handle specific packet types directly */
    if (net_pkt_family(pkt) == AF_INET) {
        LOG_INF("Handling IPv4 packet in pure NDP mode");
        /* Process packet directly without native stack */
        return NET_OK; /* Packet handled */
    }
    
    /* For other packet types, continue to native stack (if not in NDP-only mode) */
    return NET_CONTINUE;
}

/**
 * @brief Network interface event callback
 */
static void iface_cb(struct net_mgmt_event_callback *cb,
                     uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_IF_UP) {
        LOG_INF("Network interface %s is up", net_if_get_name(iface, NULL));
        
        /* Register NDP callback based on configuration */
        int ret;
        if (IS_ENABLED(CONFIG_NDP_ONLY_MODE)) {
            ret = net_ndp_register_callback(iface, ndp_pure_callback);
            LOG_INF("Pure NDP mode callback registered: %d", ret);
        } else {
            ret = net_ndp_register_callback(iface, ndp_mixed_callback);
            LOG_INF("Mixed mode NDP callback registered: %d", ret);
        }
    }
}

/**
 * @brief Main application entry point
 */
int main(void)
{
    static struct net_mgmt_event_callback mgmt_cb;
    
    LOG_INF("NDP Sample Application Started");
    
    /* Configure network interface management callback */
    net_mgmt_init_event_callback(&mgmt_cb, iface_cb, NET_EVENT_IF_UP);
    net_mgmt_add_event_callback(&mgmt_cb);
    
    /* Log configuration */
    if (IS_ENABLED(CONFIG_NDP_ONLY_MODE)) {
        LOG_INF("Running in Pure NDP mode (bypassing native stack)");
    } else {
        LOG_INF("Running in Mixed NDP mode (preprocessing + native stack)");
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