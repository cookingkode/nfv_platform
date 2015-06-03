/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#ifndef __FLOW_H_
#define __FLOW_H_

#include <stdint.h>
#include <stdbool.h>
#include <rte_mbuf.h>
#include <rte_ether.h>


#include "action.h"

/* Maximum number of flow table entries */
#define MAX_FLOWS               (1 << 16)

#define MIN_FLOWS_CLEANUP       (MAX_FLOWS )

/* Measured CPU frequency. Needed to translate tsk to ms. */
uint64_t cpu_freq;
/* Global timestamp counter that can be updated
 * only by vswitchd core. It's used as flow's last
 * used time, when next packet arrives.
 */
volatile uint64_t curr_tsc;

struct flow_key {
    uint32_t in_port;
    struct ether_addr ether_dst;
    struct ether_addr ether_src;
    uint16_t ether_type;
    uint16_t vlan_id;
    uint8_t vlan_prio;
    uint32_t ip_src;
    uint32_t ip_dst;
    uint8_t ip_proto;
    uint8_t ip_tos;
    uint8_t ip_ttl;
    uint8_t ip_frag;
    uint16_t tran_src_port;
    uint16_t tran_dst_port;
} __attribute__((__packed__));


union ipv6_addr_u {
    uint8_t s8[16];
    uint32_t s32[4];
} __attribute__((__packed__));

struct flow_key_v6 {
    uint32_t in_port;
    struct ether_addr ether_dst;
    struct ether_addr ether_src;
    uint16_t ether_type;
    uint16_t vlan_id;
    uint8_t vlan_prio;
    union ipv6_addr_u ip_src;
    union ipv6_addr_u ip_dst;
    uint8_t ip_proto;
    uint8_t ip_tos;
    uint8_t ip_ttl;
    uint8_t ip_frag;
    uint16_t tran_src_port;
    uint16_t tran_dst_port;
} __attribute__((__packed__));

struct flow_stats {
    uint64_t packet_count;	/* Number of packets matched. */
    uint64_t byte_count;	/* Number of bytes matched. */
    uint64_t used;			/* Last used time (in hpet cycles). */
    uint8_t tcp_flags;		/* Union of seen TCP flags. */
};

void flow_table_init(void);
long  flow_table_lookup(const struct flow_key *key);
void flow_key_extract(const struct rte_mbuf *pkt, uint8_t in_port,
                      struct flow_key *key);
int flow_table_del_flow(const struct flow_key *key);
void flow_table_del_all(void);
int flow_table_add_flow(const struct flow_key *key, const struct action *action,
                        uint32_t rule_id);
int flow_table_mod_flow(const struct flow_key *key, const struct action *action,
                        bool clear_stats);
int flow_table_get_flow(struct flow_key *key,
                        struct action *action, struct flow_stats *stats);
int flow_table_get_first_flow(struct flow_key *first_key,
                              struct action *action, struct flow_stats *stats);
int flow_table_get_next_flow(const struct flow_key *key,
                             struct flow_key *next_key, struct action *action,
                             struct flow_stats *stats);
void switch_packet(struct rte_mbuf *pkt, struct flow_key *key);

int vb_em_flow_table_add_flow(void *key, void *actions, uint32_t rule_id);
int vb_em_flow_table_del_flow(int flow_pos);

#endif /* __FLOW_H_ */


