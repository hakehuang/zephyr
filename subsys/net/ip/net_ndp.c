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
        if (callbacks[i].active && 
            callbacks[i].iface == iface && 
            callbacks[i].cb == cb) {
            k_mutex_unlock(&callback_mutex);
            LOG_WRN("Callback already registered");
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
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    ret = net_ndp_init();
    if (ret < 0) {
        return ret;
    }
    
    k_mutex_lock(&callback_mutex, K_FOREVER);
    
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (callbacks[i].active && 
            callbacks[i].iface == iface && 
            callbacks[i].cb == cb) {
            callbacks[i].active = false;
            callbacks[i].iface = NULL;
            callbacks[i].cb = NULL;
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
 * 
 * @param iface Network interface
 * @param pkt Packet to process
 * @return enum net_verdict Processing verdict
 */
enum net_verdict net_ndp_process_packet(struct net_if *iface, struct net_pkt *pkt)
{
    int ret;
    enum net_verdict verdict = NET_CONTINUE;
    
    if (iface == NULL || pkt == NULL) {
        return NET_DROP;
    }
    
    ret = net_ndp_init();
    if (ret < 0) {
        return NET_DROP;
    }
    
    k_mutex_lock(&callback_mutex, K_FOREVER);
    
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (callbacks[i].active && callbacks[i].iface == iface) {
            verdict = callbacks[i].cb(pkt);
            
            /* If callback returns anything other than CONTINUE, stop processing */
            if (verdict != NET_CONTINUE) {
                break;
            }
        }
    }
    
    k_mutex_unlock(&callback_mutex);
    return verdict;
}