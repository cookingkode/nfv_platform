/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef __DATAPATH_H_
#define __DATAPATH_H_

#include <rte_mbuf.h>
#include "flow.h"

enum flow_cmd {
    FLOW_CMD_UNSPEC,
    FLOW_CMD_NEW,
    FLOW_CMD_DEL,
    FLOW_CMD_GET
};

enum packet_cmd {
    PACKET_CMD_UNSPEC,
    PACKET_CMD_MISS,
    PACKET_CMD_ACTION,
    PACKET_CMD_EXECUTE
};

struct dpdk_upcall {
    uint8_t cmd;         /* The reason why we are sending the packet to
	                            the daemon. */
    struct flow_key key; /* Extracted flow key for the packet. */
};

void handle_request_from_vswitchd(void);
void send_packet_to_vswitchd(struct rte_mbuf *mbuf, struct dpdk_upcall *info);
void datapath_init(void);

#endif /* __DATAPATH_H_ */


