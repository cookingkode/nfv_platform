/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef __VPORT_H_
#define __VPORT_H_

#include <stdint.h>
#include <rte_mbuf.h>

#include "kni.h"
#include "veth.h"
#include "tap.h"

#define MAX_VPORTS          180
#define MAX_PHYPORTS        16
#define MAX_CLIENTS         16
#define PKT_BURST_SIZE      32u
#define CLIENT0             0
#define CLIENT1             1
#define PHYPORT0            0x10
#define KNI0                0x20
#define VETH0               0x40
#define TAP0                0x80
#define CLIENT_MASK         0x00
#define PORT_MASK           0x0F
#define KNI_MASK            0x1F
#define VETH_MASK           0x3F
#define IS_CLIENT_PORT(action) ((action) > CLIENT_MASK && (action) <= PORT_MASK)
#define IS_PHY_PORT(action) ((action) > PORT_MASK && (action) <= KNI_MASK)
#define IS_KNI_PORT(action) ((action) > KNI_MASK  && (action) <= (KNI_MASK + MAX_KNI_PORTS))
#define IS_VETH_PORT(action) ((action) > VETH_MASK  && (action) <= (VETH_MASK + MAX_VETH_PORTS))

struct port_info {
    uint8_t num_ports;
    uint8_t id[RTE_MAX_ETHPORTS];
};

struct port_info *ports;

void vport_init(void);
void vport_fini(void);

int send_to_vport(uint8_t vportid, struct rte_mbuf *buf);
uint16_t receive_from_vport(uint8_t vportid, struct rte_mbuf **bufs);
void flush_nic_tx_ring(unsigned vportid);
const char *vport_name(unsigned vportid);

void flush_clients(void);
void flush_ports(void);

#endif /* __VPORT_H_ */


