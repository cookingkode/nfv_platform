/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#ifndef __STATS_H_
#define __STATS_H_

#define INC_BY_1  1
#define VSWITCHD 0

void stats_init(void);
void stats_fini(void);
void stats_clear(void);

void stats_vport_clear_all(void);
void stats_vport_clear(unsigned vportid);
void stats_vport_rx_increment(unsigned vportid, int inc);
void stats_vport_rx_drop_increment(unsigned vportid, int inc);
void stats_vport_tx_increment(unsigned vportid, int inc);
void stats_vport_tx_drop_increment(unsigned vportid, int inc);
void stats_vport_overrun_increment(unsigned vportid, int inc);
uint64_t stats_vport_rx_get(unsigned vportid);
uint64_t stats_vport_rx_drop_get(unsigned vportid);
uint64_t stats_vport_tx_get(unsigned vportid);
uint64_t stats_vport_tx_drop_get(unsigned vportid);
uint64_t stats_vport_overrun_get(unsigned vportid);


void stats_vswitch_clear(void);
void stats_vswitch_rx_drop_increment(int inc);
uint64_t stats_vswitch_rx_drop_get(void);
void stats_vswitch_tx_drop_increment(int inc);
uint64_t stats_vswitch_tx_drop_get(void);


#endif /* __STATS_H_ */

