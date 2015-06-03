/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */


#ifndef __TAP_H_
#define __TAP_H_


int
init_taps();

uint16_t
receive_from_tap(uint8_t vportid, struct rte_mbuf **bufs);

int
send_to_tap(uint8_t vportid, struct rte_mbuf *buf);

#endif /* __TAP_H_ */   

