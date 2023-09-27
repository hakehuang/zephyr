/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <zephyr/net/net_ip.h>

#define PR(fmt, ...)						\
	shell_fprintf(sh, SHELL_NORMAL, fmt, ##__VA_ARGS__)

#define PR_SHELL(sh, fmt, ...)				\
	shell_fprintf(sh, SHELL_NORMAL, fmt, ##__VA_ARGS__)

#define PR_ERROR(fmt, ...)					\
	shell_fprintf(sh, SHELL_ERROR, fmt, ##__VA_ARGS__)

#define PR_INFO(fmt, ...)					\
	shell_fprintf(sh, SHELL_INFO, fmt, ##__VA_ARGS__)

#define PR_WARNING(fmt, ...)					\
	shell_fprintf(sh, SHELL_WARNING, fmt, ##__VA_ARGS__)

#include "net_private.h"

struct net_shell_user_data {
	const struct shell *sh;
	void *user_data;
};

#if !defined(NET_VLAN_MAX_COUNT)
#define MAX_IFACE_COUNT NET_IF_MAX_CONFIGS
#else
#define MAX_IFACE_COUNT NET_VLAN_MAX_COUNT
#endif

#define MAX_IFACE_HELP_STR_LEN sizeof("longbearername (0xabcd0123)")
#define MAX_IFACE_STR_LEN sizeof("xxx")

#if defined(CONFIG_NET_SHELL_DYN_CMD_COMPLETION)
#define IFACE_DYN_CMD &iface_index

void iface_index_get(size_t idx, struct shell_static_entry *entry);
#else
#define IFACE_DYN_CMD NULL
#endif /* CONFIG_NET_SHELL_DYN_CMD_COMPLETION */

const char *addrtype2str(enum net_addr_type addr_type);
const char *addrstate2str(enum net_addr_state addr_state);
void get_addresses(struct net_context *context,
		   char addr_local[], int local_len,
		   char addr_remote[], int remote_len);
void events_enable(void);
int get_iface_idx(const struct shell *sh, char *index_str);
const char *iface2str(struct net_if *iface, const char **extra);
