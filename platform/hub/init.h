/*
 *
 * Copyright 2014 Jyotiswarup Raitukar
 * All rights reserved
 */


#ifndef _INIT_H_
#define _INIT_H_

struct port_queue {
    unsigned port_id;
    struct rte_ring *tx_q;
};

struct port_queue *port_queues;


/* The mbuf pool for packet rx */
struct rte_mempool *pktmbuf_pool;
uint8_t num_clients;
uint8_t num_kni;
uint8_t num_veth;
unsigned num_sockets;
uint8_t num_tap;

unsigned stats_display_interval;
unsigned vswitchd_core;
unsigned client_switching_core;

int init(int argc, char *argv[]);

#endif /* ifndef _INIT_H_ */
