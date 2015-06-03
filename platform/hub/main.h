/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */



#ifndef _MAIN_H_
#define _MAIN_H_

#define SOCKET0 0
#define SOCKET1 1

#ifdef RTE_EXEC_ENV_BAREMETAL
#define MAIN _main
#else
#define MAIN main
#endif

int MAIN(int argc, char **argv);

#endif /* _MAIN_H_ */
