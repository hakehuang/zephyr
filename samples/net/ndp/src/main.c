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
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>

LOG_MODULE_REGISTER(ndp_sample, LOG_LEVEL_INF);

#define PERF_TEST_INTERVAL_MS 1000
#define PERF_TEST_PACKET_SIZE_MIN 64
#define PERF_TEST_PACKET_SIZE_MAX 1518

/**
 * @brief Performance test statistics structure
 */
struct perf_stats {
    uint64_t packets_processed;
    uint64_t packets_dropped;
    uint64_t bytes_processed;
    uint64_t processing_time_us;
    uint64_t min_latency_us;
    uint64_t max_latency_us;
    uint64_t total_latency_us;
    uint32_t packet_sizes[PERF_TEST_PACKET_SIZE_MAX + 1];
    uint32_t ipv4_packets;
    uint32_t ipv6_packets;
    k_timeout_t test_duration;
    uint64_t test_start_time;
    bool test_running;
};

/**
 * @brief Global performance statistics instance
 */
static struct perf_stats g_perf_stats = {
    .min_latency_us = UINT64_MAX,
    .packets_processed = 0,
    .packets_dropped = 0,
    .bytes_processed = 0,
    .processing_time_us = 0,
    .total_latency_us = 0,
    .ipv4_packets = 0,
    .ipv6_packets = 0,
    .test_running = false,
};

/**
 * @brief Initialize performance statistics
 */
static void perf_stats_init(struct perf_stats *stats, k_timeout_t duration)
{
    memset(stats, 0, sizeof(struct perf_stats));
    stats->min_latency_us = UINT64_MAX;
    stats->test_duration = duration;
    stats->test_start_time = k_uptime_ticks();
    stats->test_running = true;
}

/**
 * @brief Record a packet processing event with timing
 */
static void perf_record_packet(struct perf_stats *stats, struct net_pkt *pkt,
                                uint64_t processing_time_us)
{
    uint16_t pkt_len = net_pkt_get_len(pkt);

    stats->packets_processed++;
    stats->bytes_processed += pkt_len;
    stats->processing_time_us += processing_time_us;
    stats->total_latency_us += processing_time_us;

    if (processing_time_us < stats->min_latency_us) {
        stats->min_latency_us = processing_time_us;
    }
    if (processing_time_us > stats->max_latency_us) {
        stats->max_latency_us = processing_time_us;
    }

    if (pkt_len <= PERF_TEST_PACKET_SIZE_MAX) {
        stats->packet_sizes[pkt_len]++;
    }

    if (net_pkt_family(pkt) == AF_INET) {
        stats->ipv4_packets++;
    } else if (net_pkt_family(pkt) == AF_INET6) {
        stats->ipv6_packets++;
    }
}

/**
 * @brief Record a dropped packet event
 */
static void perf_record_dropped(struct perf_stats *stats)
{
    stats->packets_dropped++;
}

/**
 * @brief Calculate and return current throughput in bits per second
 */
static uint64_t perf_calc_throughput_bps(struct perf_stats *stats)
{
    uint64_t elapsed_ms = k_uptime_delta(&stats->test_start_time);
    if (elapsed_ms == 0) {
        return 0;
    }
    return (stats->bytes_processed * 8 * 1000) / elapsed_ms;
}

/**
 * @brief Calculate and return current packets per second
 */
static uint64_t perf_calc_pps(struct perf_stats *stats)
{
    uint64_t elapsed_ms = k_uptime_delta(&stats->test_start_time);
    if (elapsed_ms == 0) {
        return 0;
    }
    return (stats->packets_processed * 1000) / elapsed_ms;
}

/**
 * @brief Calculate average latency in microseconds
 */
static uint64_t perf_calc_avg_latency_us(struct perf_stats *stats)
{
    if (stats->packets_processed == 0) {
        return 0;
    }
    return stats->total_latency_us / stats->packets_processed;
}

/**
 * @brief Calculate packet drop rate as percentage
 */
static uint64_t perf_calc_drop_rate(struct perf_stats *stats)
{
    uint64_t total = stats->packets_processed + stats->packets_dropped;
    if (total == 0) {
        return 0;
    }
    return (stats->packets_dropped * 100) / total;
}

/**
 * @brief Print performance test report
 */
static void perf_print_report(struct perf_stats *stats)
{
    uint64_t throughput_bps = perf_calc_throughput_bps(stats);
    uint64_t pps = perf_calc_pps(stats);
    uint64_t avg_latency_us = perf_calc_avg_latency_us(stats);
    uint64_t drop_rate = perf_calc_drop_rate(stats);

    LOG_INF("========== NDP Performance Test Report ==========");
    LOG_INF("Total Packets Processed: %llu", stats->packets_processed);
    LOG_INF("Total Packets Dropped: %llu", stats->packets_dropped);
    LOG_INF("Total Bytes Processed: %llu", stats->bytes_processed);
    LOG_INF("IPv4 Packets: %u", stats->ipv4_packets);
    LOG_INF("IPv6 Packets: %u", stats->ipv6_packets);
    LOG_INF("--------------------------------------------");
    LOG_INF("Throughput: %llu.%llu Mbps",
           throughput_bps / 1000000,
           (throughput_bps % 1000000) / 1000);
    LOG_INF("Packets Per Second: %llu pps", pps);
    LOG_INF("--------------------------------------------");
    LOG_INF("Min Latency: %llu us", stats->min_latency_us == UINT64_MAX ? 0 : stats->min_latency_us);
    LOG_INF("Max Latency: %llu us", stats->max_latency_us);
    LOG_INF("Avg Latency: %llu us", avg_latency_us);
    LOG_INF("--------------------------------------------");
    LOG_INF("Packet Drop Rate: %llu%%", drop_rate);
    LOG_INF("============================================");
}

/**
 * @brief Periodic performance reporting task
 */
static void perf_report_task(struct k_work *work)
{
    extern struct perf_stats g_perf_stats;
    struct perf_stats *stats = &g_perf_stats;

    if (stats->test_running && stats->packets_processed > 0) {
        perf_print_report(stats);
    }

    k_work_reschedule((struct k_work_delayable *)work,
                     K_MSEC(PERF_TEST_INTERVAL_MS));
}

static K_WORK_DELAYABLE_DEFINE(perf_report_work, perf_report_task);

/**
 * @brief Start performance test with specified duration
 */
static void perf_test_start(k_timeout_t duration)
{
    LOG_INF("Starting NDP Performance Test (duration: %lld ms)", k_ticks_to_ms_floor64(duration.ticks));
    perf_stats_init(&g_perf_stats, duration);
    k_work_reschedule(&perf_report_work, K_MSEC(100));
}

/**
 * @brief Stop performance test and print final report
 */
static void perf_test_stop(void)
{
    g_perf_stats.test_running = false;
    k_work_cancel_delayable(&perf_report_work);
    LOG_INF("NDP Performance Test Stopped");
    perf_print_report(&g_perf_stats);
}

/**
 * @brief NDP callback function for packet processing (Mixed Mode)
 */
static enum net_verdict ndp_mixed_callback(struct net_pkt *pkt)
{
    int64_t start_time = k_uptime_ticks();

    LOG_DBG("NDP Mixed Mode: Processing packet");

    if (net_pkt_family(pkt) == AF_INET) {
        LOG_DBG("IPv4 packet");
    } else if (net_pkt_family(pkt) == AF_INET6) {
        LOG_DBG("IPv6 packet");
    }

    if (g_perf_stats.test_running) {
        uint64_t processing_time_us = k_ticks_to_us_ceil64(k_uptime_ticks() - start_time);
        perf_record_packet(&g_perf_stats, pkt, processing_time_us);
    }

    return NET_CONTINUE;
}

/**
 * @brief NDP callback function for packet processing (Pure NDP Mode)
 */
static enum net_verdict ndp_pure_callback(struct net_pkt *pkt)
{
    int64_t start_time = k_uptime_ticks();

    LOG_DBG("NDP Pure Mode: Processing packet");

    if (net_pkt_family(pkt) == AF_INET) {
        LOG_DBG("Handling IPv4 packet in pure NDP mode");
        if (g_perf_stats.test_running) {
            uint64_t processing_time_us = k_ticks_to_us_ceil64(k_uptime_ticks() - start_time);
            perf_record_packet(&g_perf_stats, pkt, processing_time_us);
        }
        return NET_OK;
    }

    if (g_perf_stats.test_running) {
        uint64_t processing_time_us = k_ticks_to_us_ceil64(k_uptime_ticks() - start_time);
        perf_record_packet(&g_perf_stats, pkt, processing_time_us);
    }

    return NET_CONTINUE;
}

/**
 * @brief Network interface event callback
 */
static void iface_cb(struct net_mgmt_event_callback *cb,
                     uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_IF_UP) {
        char ifname[32];
        net_if_get_name(iface, ifname, sizeof(ifname));
        LOG_INF("Network interface %s is up", ifname);
        
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

    net_mgmt_init_event_callback(&mgmt_cb, iface_cb, NET_EVENT_IF_UP);
    net_mgmt_add_event_callback(&mgmt_cb);

    if (IS_ENABLED(CONFIG_NDP_ONLY_MODE)) {
        LOG_INF("Running in Pure NDP mode (bypassing native stack)");
    } else {
        LOG_INF("Running in Mixed NDP mode (preprocessing + native stack)");
    }

    perf_test_start(K_SECONDS(60));

    while (1) {
        static uint32_t counter = 0;
        LOG_INF("NDP sample running for %u seconds", counter++);

        k_sleep(K_SECONDS(5));
    }

    perf_test_stop();
    return 0;
}
