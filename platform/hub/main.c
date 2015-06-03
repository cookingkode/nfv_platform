/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#include <rte_ethdev.h>
#include <rte_kni.h>
#include <rte_cycles.h>
#include <rte_string_fns.h>
#include <rte_config.h>

#include "kni.h"
#include "veth.h"
#include "args.h"
#include "init.h"
#include "main.h"
#include "vport.h"
#include "stats.h"
#include "flow.h"
#include "datapath.h"
#include "action.h"
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#define RTE_LOGTYPE_APP RTE_LOGTYPE_USER1
#define NUM_BYTES_MAC_ADDR  6
#define MAC_ADDR_STR_INT_L  3
#define NEWLINE_CHAR_OFFSET 1

/*
 * When reading/writing to/from rings/ports use this batch size
 */
#define PREFETCH_OFFSET     3
#define RX_RING_SIZE        128
#define TX_RING_SIZE        512
#define RING_SIZE           (PKT_BURST_SIZE * 8)
#define BYTES_TO_PRINT      256
#define BURST_TX_DRAIN_US   (100) /* TX drain every ~100us */
#define RUN_ON_THIS_THREAD  1


static const char *get_printable_mac_addr(uint8_t port);
static void stats_display(void);

/*
 * Returns MAC address for port in a string
 */
static const char *
get_printable_mac_addr(uint8_t port)
{
    static const char err_address[] = "00:00:00:00:00:00";
    static char addresses[RTE_MAX_ETHPORTS][sizeof(err_address)] = {{0}};
    struct ether_addr mac = {{0}};
    int ether_addr_len = sizeof(err_address) - NEWLINE_CHAR_OFFSET;
    int i = 0;
    int j = 0;

    if (unlikely(port >= RTE_MAX_ETHPORTS))
        return err_address;

    /* first time run for this port so we populate addresses */
    if (unlikely(addresses[port][0] == '\0')) {
        rte_eth_macaddr_get(port, &mac);
        while(j < NUM_BYTES_MAC_ADDR) {
            rte_snprintf(&addresses[port][0] + i,
                         MAC_ADDR_STR_INT_L + NEWLINE_CHAR_OFFSET,
                         "%02x:",
                         mac.addr_bytes[j]);
            i += MAC_ADDR_STR_INT_L;
            j++;
        }
        /* Overwrite last ":" and null terminate the string */
        addresses[port][ether_addr_len] = '\0';
    }
    return addresses[port];
}

/*
 * This function displays the recorded statistics for each port
 * and for each client. It uses ANSI terminal codes to clear
 * screen when called. It is called from a single non-master
 * thread in the server process, when the process is run with more
 * than one lcore enabled.
 */
static void
stats_display(void)
{
    unsigned i = 0;
    /* ANSI escape sequences for terminal display.
     * 27 = ESC, 2J = Clear screen */
    const char clr[] = {27, '[', '2', 'J', '\0'};
    /* H = Home position for cursor*/
    const char topLeft[] = {27, '[', '1', ';', '1', 'H','\0'};
    uint64_t overruns = 0;

    /* Clear screen and move to top left */
    printf("%s%s", clr, topLeft);

    printf("SmartTOR Hub Stats\n");
    printf("-----\n");
    for (i = 0; i < ports->num_ports; i++)
        printf("Port %u: '%s'\t", ports->id[i],
               get_printable_mac_addr(ports->id[i]));
    printf("\n\n");

    printf("\nVport Statistics\n"
           "=============   ============  ============  ============  ============\n"
           "Interface       rx_packets    rx_dropped    tx_packets    tx_dropped  \n"
           "-------------   ------------  ------------  ------------  ------------\n");
    for (i = 0; i < 130; i++) {
        const char *name = vport_name(i);
        if (*name == 0)
            continue;
        printf("%-12s ", name);
        printf("%13"PRIu64" %13"PRIu64" %13"PRIu64" %13"PRIu64"\n",
               stats_vport_rx_get(i),
               stats_vport_rx_drop_get(i),
               stats_vport_tx_get(i),
               stats_vport_tx_drop_get(i));

        overruns += stats_vport_overrun_get(i);
    }
    printf("============   ============  ============  ============  ============\n");

    printf("\n Switch rx dropped %"PRIu64"\n", stats_vswitch_rx_drop_get());
    printf("\n Switch tx dropped %"PRIu64"\n", stats_vswitch_tx_drop_get());
    printf("\n Queue overruns   %"PRIu64"\n",  overruns);
    printf("\n Packet buffer count    %9"PRIu32"\n", rte_mempool_count(pktmbuf_pool));
    printf("\n");
}


static inline void
do_vswitchd(void)
{
    static uint64_t last_stats_display_tsc = 0;

    /* handle any packets from vswitchd */
    handle_request_from_vswitchd();

    /* display stats every 'stats' sec */
    curr_tsc = rte_rdtsc();
    if ((curr_tsc - last_stats_display_tsc) / cpu_freq >= stats_display_interval
        && stats_display_interval != 0) {
        last_stats_display_tsc = curr_tsc;
        stats_display();
    }
    flush_clients();
    flush_ports();
}

static inline void __attribute__((always_inline))
do_switch_packets(unsigned vportid, struct rte_mbuf **bufs, int rx_count)
{
    int j;
    struct flow_key key;

    /* Prefetch first packets */
    for (j = 0; j < PREFETCH_OFFSET && j < rx_count; j++)
        rte_prefetch0(rte_pktmbuf_mtod(bufs[j], void *));

    /* Prefetch new packets and forward already prefetched packets */
    for (j = 0; j < (rx_count - PREFETCH_OFFSET); j++) {
        flow_key_extract(bufs[j], vportid, &key);
        rte_prefetch0(rte_pktmbuf_mtod(bufs[j + PREFETCH_OFFSET], void *));
        switch_packet(bufs[j], &key);
    }

    /* Forward remaining prefetched packets */
    for (; j < rx_count; j++) {
        flow_key_extract(bufs[j], vportid, &key);
        switch_packet(bufs[j], &key);
    }
}

static inline void __attribute__((always_inline))
do_client_switching(void)
{
    static unsigned client = CLIENT1;
    static unsigned kni_vportid = KNI0;
    static unsigned veth_vportid = VETH0;
    static unsigned tap_vportid = TAP0;
    int rx_count = 0;
    struct rte_mbuf *bufs[PKT_BURST_SIZE];

    /* Client ports */

    rx_count = receive_from_vport(client, &bufs[0]);
    do_switch_packets(client, bufs, rx_count);


    /* move to next client and dont handle client 0*/
    if (++client == num_clients) {
        client = 1;
    }

    /* KNI ports */
    if (num_kni) {
        rx_count = receive_from_vport(kni_vportid, &bufs[0]);
        do_switch_packets(kni_vportid, bufs, rx_count);

        /* move to next kni port */
        if (++kni_vportid == (unsigned)KNI0 + num_kni) {
            kni_vportid = KNI0;
        }
    }

    if (num_tap) {
        rx_count = receive_from_vport(tap_vportid, &bufs[0]);
        do_switch_packets(tap_vportid, bufs, rx_count);

        /* move to next tap port */
        if (++tap_vportid == (unsigned)TAP0 + num_tap) {
            tap_vportid = TAP0;
        }
    }

    /* vETH Devices */
    if (unlikely(num_veth)) {  /* vEth devices are optional */
        rx_count = receive_from_vport(veth_vportid, &bufs[0]);
        do_switch_packets(veth_vportid, bufs, rx_count);

        /* move to next veth port */
        if (++veth_vportid == (unsigned)VETH0 + num_veth) {
            veth_vportid = VETH0;
        }
    }

    flush_clients();
    flush_ports();
}

static inline void __attribute__((always_inline))
do_port_switching(unsigned vportid)
{
    int rx_count = 0;
    struct rte_mbuf *bufs[PKT_BURST_SIZE];

    rx_count = receive_from_vport(vportid, &bufs[0]);
    do_switch_packets(vportid, bufs, rx_count);

    flush_clients();
    flush_ports();
    flush_nic_tx_ring(vportid);
}

/* Get CPU frequency */
static void
measure_cpu_frequency(void)
{
    uint64_t before = 0;
    uint64_t after = 0;

    /* How TSC changed in 1 second - it is the CPU frequency */
    before = rte_rdtsc();
    sleep(1);
    after = rte_rdtsc();
    cpu_freq = after - before;

    /* Round to millions */
    cpu_freq /= 1000000;
    cpu_freq *= 1000000;
}

/* Main function used by the processing threads.
 * Prints out some configuration details for the thread and then begins
 * performing packet RX and TX.
 */
static int
lcore_main(void *arg __rte_unused)
{
    unsigned i = 0;
    const unsigned id = rte_lcore_id();
    unsigned nr_vswitchd = 0;
    unsigned nr_client_switching = 0;
    unsigned nr_port_switching = 0;
    unsigned portid_map[MAX_PHYPORTS] = {0};

    /* vswitchd core is used for print_stat and receive_from_vswitchd */
    if (id == vswitchd_core) {
        RTE_LOG(INFO, APP, "Print stat core is %d.\n", id);

        /* Measuring CPU frequency */
        measure_cpu_frequency();
        RTE_LOG(INFO, APP, "CPU frequency is %"PRIu64" MHz\n", cpu_freq / 1000000);
        nr_vswitchd = RUN_ON_THIS_THREAD;
    }
    /* client_switching_core is used process packets from client rings
     * or fifos
     */

    if (id == client_switching_core) {
        RTE_LOG(INFO, APP, "Client switching core is %d.\n", id);
        nr_client_switching = RUN_ON_THIS_THREAD;
    }

    for (i = 0; i < nb_cfg_params; i++) {
        if (id == cfg_params[i].lcore_id) {
            RTE_LOG(INFO, APP, "Port core is %d.\n", id);
            portid_map[nr_port_switching++] = cfg_params[i].port_id;
        }
    }


    for (;;) {
        if (nr_vswitchd) {
            do_vswitchd();
            //do_cli_getmsg();
        }
        if (nr_client_switching) {
            do_client_switching();
        }

        for (i = 0; i < nr_port_switching; i++)
            do_port_switching(portid_map[i]);
    }

    return 0;
}


int
MAIN(int argc, char *argv[])
{
    unsigned i = 0;
    struct rlimit core_limit;
    core_limit.rlim_cur = RLIM_INFINITY;
    core_limit.rlim_max = RLIM_INFINITY;

    if (setrlimit(RLIMIT_CORE, &core_limit) < 0) {
        RTE_LOG(INFO, APP, "Process RLIMIT failed  failed.\n");
        return -1;
    }

    if (init(argc, argv) < 0 ) {
        RTE_LOG(INFO, APP, "Process init failed.\n");
        return -1;
    }

    RTE_LOG(INFO, APP, "Finished Process Init.\n");

#if 0
    if (RTE_PROC_SECONDARY != rte_eal_process_type()) {
        while (1) {
            sleep(1000);
        }
    }
#endif
    stats_clear();

    for (i = 0; i < nb_cfg_params; i++) {
        RTE_LOG(INFO, APP, "config = %d,%d,%d\n",
                cfg_params[i].port_id,
                cfg_params[i].queue_id,
                cfg_params[i].lcore_id);
    }
    RTE_LOG(INFO, APP, "nb_cfg_params = %d\n", nb_cfg_params);

    rte_eal_mp_remote_launch(lcore_main, NULL, CALL_MASTER);
    RTE_LCORE_FOREACH_SLAVE(i) {
        if (rte_eal_wait_lcore(i) < 0)
            return -1;
    }
    return 0;
}
