/*
 * Copyright NDP Developer 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_ndp.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_ndp, CONFIG_NDP_LOG_LEVEL);

#define MAX_CALLBACKS 10

struct ndp_callback_entry {
    struct net_if *iface;
    enum net_verdict (*cb)(struct net_pkt *);
    bool active;
};

static struct ndp_callback_entry callbacks[MAX_CALLBACKS];
static struct k_mutex callback_mutex;

/**
 * @brief Initialize NDP callback system
 */
static int net_ndp_init(void)
{
    static bool initialized = false;
    
    if (!initialized) {
        k_mutex_init(&callback_mutex);
        memset(callbacks, 0, sizeof(callbacks));
        initialized = true;
        /* NDP callback system initialized */
    }
    
    return 0;
}

/**
 * @brief Register a callback for NDP packet processing
 */
int net_ndp_register_callback(struct net_if *iface,
                             enum net_verdict (*cb)(struct net_pkt *))
{
    int ret;
    
    if (iface == NULL || cb == NULL) {
        return -EINVAL;
    }
    
    ret = net_ndp_init();
    if (ret < 0) {
        return ret;
    }
    
    k_mutex_lock(&callback_mutex, K_FOREVER);
    
    /* Check if callback already registered */
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (callbacks[i].active && callbacks[i].iface == iface && 
            callbacks[i].cb == cb) {
            k_mutex_unlock(&callback_mutex);
            return -EALREADY;
        }
    }
    
    /* Find free slot */
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (!callbacks[i].active) {
            callbacks[i].iface = iface;
            callbacks[i].cb = cb;
            callbacks[i].active = true;
            k_mutex_unlock(&callback_mutex);
            
            /* NDP callback registered */
            return 0;
        }
    }
    
    k_mutex_unlock(&callback_mutex);
    /* No free slots for NDP callback */
    return -ENOMEM;
}

/**
 * @brief Unregister a callback from NDP packet processing
 */
int net_ndp_unregister_callback(struct net_if *iface,
                               enum net_verdict (*cb)(struct net_pkt *))
{
    int ret;
    
    if (iface == NULL || cb == NULL) {
        return -EINVAL;
    }
    
    ret = net_ndp_init();
    if (ret < 0) {
        return ret;
    }
    
    k_mutex_lock(&callback_mutex, K_FOREVER);
    
    /* Find and remove callback */
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (callbacks[i].active && callbacks[i].iface == iface && 
            callbacks[i].cb == cb) {
            callbacks[i].active = false;
            k_mutex_unlock(&callback_mutex);
            
            /* NDP callback unregistered */
            return 0;
        }
    }
    
    k_mutex_unlock(&callback_mutex);
    return -ENOENT;
}

/**
 * @brief Process packet through registered NDP callbacks
 */
enum net_verdict net_ndp_process_packet(struct net_if *iface, struct net_pkt *pkt)
{
    int ret;
    
    if (iface == NULL || pkt == NULL) {
        return NET_CONTINUE;
    }
    
    ret = net_ndp_init();
    if (ret < 0) {
        return NET_CONTINUE;
    }
    
    k_mutex_lock(&callback_mutex, K_FOREVER);
    
    /* Process through all registered callbacks for this interface */
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (callbacks[i].active && callbacks[i].iface == iface) {
            enum net_verdict verdict = callbacks[i].cb(pkt);
            
            /* If callback returns DROP or OK, return immediately */
            if (verdict == NET_DROP || verdict == NET_OK) {
                k_mutex_unlock(&callback_mutex);
                return verdict;
            }
            
            /* If callback returns CONTINUE, continue to next callback */
        }
    }
    
    k_mutex_unlock(&callback_mutex);
    return NET_CONTINUE;
}