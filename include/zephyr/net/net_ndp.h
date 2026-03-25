/*
 * Copyright NDP Developer 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Use the existing net_verdict enum from net_core.h */
#include <zephyr/net/net_core.h>

/**
 * @brief Register a callback for NDP packet processing
 * 
 * @param iface Network interface to register callback on
 * @param cb Callback function for packet processing
 * @return int 0 on success, negative error code on failure
 */
int net_ndp_register_callback(struct net_if *iface,
                             enum net_verdict (*cb)(struct net_pkt *));

/**
 * @brief Unregister a callback from NDP packet processing
 * 
 * @param iface Network interface to unregister callback from
 * @param cb Callback function to unregister
 * @return int 0 on success, negative error code on failure
 */
int net_ndp_unregister_callback(struct net_if *iface,
                               enum net_verdict (*cb)(struct net_pkt *));

/**
 * @brief Process packet through registered NDP callbacks
 * 
 * @param iface Network interface
 * @param pkt Packet to process
 * @return enum net_verdict Processing verdict
 */
enum net_verdict net_ndp_process_packet(struct net_if *iface, struct net_pkt *pkt);

#ifdef __cplusplus
}
#endif