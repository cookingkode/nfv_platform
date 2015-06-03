/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef __VETH_H_
#define __VETH_H_

#define MAX_VETH_PORTS         4

struct rte_kni *rte_veth_list[MAX_VETH_PORTS];

/* Reserves memory for MAX_VETH_PORTS number of KNI ports and initialises
 * the fifos
 */
void
init_veth(void);

#endif
