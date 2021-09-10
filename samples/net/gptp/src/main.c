/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_gptp_sample, LOG_LEVEL_DBG);

#include <zephyr.h>
#include <errno.h>

#include <net/net_core.h>
#include <net/net_l2.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <net/gptp.h>

/*USER BEGIN INCLUDES*/
#include <net/ptp_time.h>
#include <sys/printk.h>
#include <sys/util.h>
/*USER END INCLUDES*/


/*USER BEGIN VARIABLES*/
static struct net_ptp_time slave_time;
//struct gptp_clk_src_time_invoke_params src_time_invoke_parameters;
bool gm_present;
int status;
/*USER END VARIABLES*/

extern void init_testing(void);

static struct gptp_phase_dis_cb phase_dis;

#if defined(CONFIG_NET_GPTP_VLAN)
/* User data for the interface callback */
struct ud {
	struct net_if *first;
	struct net_if *second;
	struct net_if *third;
};

static void iface_cb(struct net_if *iface, void *user_data)
{
	struct ud *ud = user_data;

	if (net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET)) {
		return;
	}

	if (!ud->first) {
		ud->first = iface;
		return;
	}

	if (!ud->second) {
		ud->second = iface;
		return;
	}

	if (!ud->third) {
		ud->third = iface;
		return;
	}
}

static int setup_iface(struct net_if *iface, const char *ipv6_addr,
		       const char *ipv4_addr, uint16_t vlan_tag)
{
	struct net_if_addr *ifaddr;
	struct in_addr addr4;
	struct in6_addr addr6;
	int ret;

	ret = net_eth_vlan_enable(iface, vlan_tag);
	if (ret < 0) {
		LOG_ERR("Cannot enable VLAN for tag %d (%d)", vlan_tag, ret);
	}

	if (net_addr_pton(AF_INET6, ipv6_addr, &addr6)) {
		LOG_ERR("Invalid address: %s", ipv6_addr);
		return -EINVAL;
	}

	ifaddr = net_if_ipv6_addr_add(iface, &addr6, NET_ADDR_MANUAL, 0);
	if (!ifaddr) {
		LOG_ERR("Cannot add %s to interface %p", ipv6_addr, iface);
		return -EINVAL;
	}

	if (net_addr_pton(AF_INET, ipv4_addr, &addr4)) {
		LOG_ERR("Invalid address: %s", ipv4_addr);
		return -EINVAL;
	}

	ifaddr = net_if_ipv4_addr_add(iface, &addr4, NET_ADDR_MANUAL, 0);
	if (!ifaddr) {
		LOG_ERR("Cannot add %s to interface %p", ipv4_addr, iface);
		return -EINVAL;
	}

	LOG_DBG("Interface %p VLAN tag %d setup done.", iface, vlan_tag);

	return 0;
}

static int init_vlan(void)
{
	struct ud ud;
	int ret;

	(void)memset(&ud, 0, sizeof(ud));

	net_if_foreach(iface_cb, &ud);

	/* This sample has two VLANs. For the second one we need to manually
	 * create IP address for this test. But first the VLAN needs to be
	 * added to the interface so that IPv6 DAD can work properly.
	 */
	ret = setup_iface(ud.second,
			  CONFIG_NET_SAMPLE_IFACE2_MY_IPV6_ADDR,
			  CONFIG_NET_SAMPLE_IFACE2_MY_IPV4_ADDR,
			  CONFIG_NET_SAMPLE_IFACE2_VLAN_TAG);
	if (ret < 0) {
		return ret;
	}

	ret = setup_iface(ud.third,
			  CONFIG_NET_SAMPLE_IFACE3_MY_IPV6_ADDR,
			  CONFIG_NET_SAMPLE_IFACE3_MY_IPV4_ADDR,
			  CONFIG_NET_SAMPLE_IFACE3_VLAN_TAG);
	if (ret < 0) {
		return ret;
	}

	return 0;
}
#endif /* CONFIG_NET_GPTP_VLAN */

static void gptp_phase_dis_cb(uint8_t *gm_identity,
			      uint16_t *time_base,
			      struct gptp_scaled_ns *last_gm_ph_change,
			      double *last_gm_freq_change)
{
	char output[sizeof("xx:xx:xx:xx:xx:xx:xx:xx")];
	static uint8_t id[8];

	if (memcmp(id, gm_identity, sizeof(id))) {
		memcpy(id, gm_identity, sizeof(id));

		LOG_DBG("GM %s last phase %d.%" PRId64 "",
			log_strdup(gptp_sprint_clock_id(gm_identity, output,
							sizeof(output))),
			last_gm_ph_change->high,
			last_gm_ph_change->low);
	}
}

static int init_app(void)
{
#if defined(CONFIG_NET_GPTP_VLAN)
	if (init_vlan() < 0) {
		LOG_ERR("Cannot setup VLAN");
	}
#endif

	gptp_register_phase_dis_cb(&phase_dis, gptp_phase_dis_cb);

	return 0;
}

void main(void)
{
  uint64_t prevsecond = 0;
  uint32_t prevnanosecond = 0;

  init_app();
  init_testing();

  /* USER BEGIN MAIN.C*/
  while(1){

      status=gptp_event_capture(&slave_time, &gm_present);

      //LOG_INF("gPTP time %u.%u", slave_time.second, slave_time.nanosecond);
      if ( slave_time.second == prevsecond ) {
          if (slave_time.nanosecond != prevnanosecond)
              LOG_ERR("gPTP time ERROR: %u.%u != %u.%u", prevsecond, prevnanosecond, slave_time.second, slave_time.nanosecond);
          else
              LOG_WRN("gPTP time ERROR: %u.%u == %u.%u", prevsecond, prevnanosecond, slave_time.second, slave_time.nanosecond);
      }
      prevsecond = slave_time.second;
      prevnanosecond = slave_time.nanosecond;
      k_msleep(1000); //sleep time in ms
  }
  /* USER END MAIN.C*/
}
