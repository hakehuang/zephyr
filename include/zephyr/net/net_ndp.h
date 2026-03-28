/*
 * Copyright NDP Developer 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_NDP_H_
#define ZEPHYR_INCLUDE_NET_NDP_H_

#include <zephyr/net/net_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register a callback for NDP packet processing
 * 
 * @param iface Network interface
 * @param cb Callback function
 * @return 0 on success, negative error code on failure
 */
int net_ndp_register_callback(struct net_if *iface,
                             enum net_verdict (*cb)(struct net_pkt *));

/**
 * @brief Unregister a callback from NDP packet processing
 * 
 * @param iface Network interface to unregister callback from
 * @param cb Callback function to unregister
 * @return int 0 on success, negative error code on failure
=======
 * @param iface Network interface
 * @param cb Callback function
 * @return 0 on success, negative error code on failure
>>>>>>> github/main
 */
int net_ndp_unregister_callback(struct net_if *iface,
                               enum net_verdict (*cb)(struct net_pkt *));

/**
 * @brief Process packet through registered NDP callbacks
 * 
 * @param iface Network interface
<<<<<<< HEAD
 * @param pkt Packet to process
 * @return enum net_verdict Processing verdict
=======
 * @param pkt Network packet
 * @param iface Network interface
 * @param cb Callback function
 * @return 0 on success, negative error code on failure
 */
int net_ndp_unregister_callback(struct net_if *iface,
                               enum net_verdict (*cb)(struct net_pkt *));

/**
 * @brief Process packet through registered NDP callbacks
 * 
 * @param iface Network interface
 * @param pkt Network packet
 * @return Net verdict (NET_OK, NET_DROP, NET_CONTINUE)
 */
enum net_verdict net_ndp_process_packet(struct net_if *iface, struct net_pkt *pkt);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_NET_NDP_H_ */
