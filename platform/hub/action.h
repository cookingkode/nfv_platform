/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef __ACTION_H_
#define __ACTION_H_

#include <stdint.h>

#include <rte_mbuf.h>
#include <linux/openvswitch.h>

/* TODO: same value as VPORTS increase if required */
#define MAX_ACTIONS	(48)

/* Set of all supported actions */
enum action_type {
    ACTION_NULL,    /* Empty action ; drop packet*/
    ACTION_OUTPUT,  /* Output packet to port */
    ACTION_POP_VLAN, /* Remove 802.1Q header */
    ACTION_PUSH_VLAN,/* Add 802.1Q VLAN header to packet */
    ACTION_SET_ETHERNET, /* Modify Ethernet header */
    ACTION_SET_IPV4, /* Modify IPV4 header */
    ACTION_SET_TCP, /* Modify TCP header */
    ACTION_SET_UDP, /* Modify UDP header */
    ACTION_MAX      /* Maximum number of supported actions */
};

struct action_output {
    uint32_t port;    /* Output port */
};

struct action_push_vlan {
    uint16_t tpid; /* Tag Protocol ID (always 0x8100) */
    uint16_t tci;  /* Tag Control Information */
};

struct action {
    enum action_type type;
    union { /* union of difference action types */
        struct action_output output;
        struct action_push_vlan vlan;
        struct ovs_key_ethernet ethernet;
        struct ovs_key_ipv4 ipv4;
        struct ovs_key_tcp tcp;
        struct ovs_key_udp udp;
        /* add other action structs here */
    } data;
};


int action_execute(const struct action *action, struct rte_mbuf *mbuf);

#endif /* __ACTION_H_ */


