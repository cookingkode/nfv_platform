#ifndef VB_ROUTE_H
#define VB_ROUTE_H
#include <rte_common.h>
#include <rte_mbuf.h>
#include <rte_ip.h>
#include <rte_ether.h>



void  
vb_route( struct rte_mbuf * m, uint8_t* vport,  struct ether_addr dest_addr);

#endif /*VB_ROUTE_H*/
