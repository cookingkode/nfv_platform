/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef _ARGS_H_
#define _ARGS_H_

#include "vport.h"

#define PARAM_CONFIG "config"
#define PARAM_CLIENT_CONFIG "client_config"
#define PARAM_STATS "stats"
#define PARAM_VSWITCHD "vswitchd"
#define PARAM_CSC "client_switching_core"
#define PARAM_KSC "kni_switching_core"

#define MAX_CFG_PARAMS MAX_PHYPORTS
struct cfg_params {
    uint8_t port_id;
    uint8_t queue_id;
    uint8_t lcore_id;
} __rte_cache_aligned;

extern struct cfg_params *cfg_params;
extern uint16_t nb_cfg_params;
extern int8_t client_cfg_array[MAX_CLIENTS + 1] ;

int parse_app_args(uint8_t max_ports, int argc, char *argv[]);
int parse_config(const char *q_arg);

/* global var for number of clients - extern in header */
uint8_t num_clients;
uint8_t num_kni;
unsigned stats_display_interval; /* in seconds, set to 0 to disable update */
unsigned vswitchd_core;
unsigned client_switching_core;
struct port_info port_cfg;

#endif /* ifndef _ARGS_H_ */
