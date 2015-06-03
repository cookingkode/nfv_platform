/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#ifndef _INIT_DRIVERS_H_
#define _INIT_DRIVERS_H_

/**
 * Initialise all 1G and 10G NICs available
 */
static inline int
init_drivers(void)
{
    if (
#ifdef RTE_LIBRTE_IGB_PMD
        (rte_igb_pmd_init() < 0) ||
#endif
#ifdef RTE_LIBRTE_IXGBE_PMD
        (rte_ixgbe_pmd_init() < 0) ||
#endif
        (rte_eal_pci_probe() < 0 ))
        return -1;

    return 0;
}

#endif
