/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef _LOG_VS_H
#define _LOG_VS_H

#include <limits.h>
#include <unistd.h>
#include <syslog.h>

#define VS_LOG_LEVEL_DEBUG   LOG_DEBUG
#define VS_LOG_LEVEL_CRIT    LOG_CRIT


__thread  int log_vs_fd = 0;
__thread  char log_vs_buf[PIPE_BUF] = { '\0' };
__thread int syslog_opened = 0;



#define log_vs(level,msg) \
{ \
  	time_t tm = time(NULL); \
  	struct tm the_date; \
	localtime_r(&tm, &the_date); \
	\
	if(!syslog_opened) { \
		openlog("[ DSE_LOG ]", LOG_PID, LOG_USER);\
		syslog_opened = 1; \
	} \
	 \
	syslog(LOG_DAEMON|level, "%.2d/%.2d/%.4d %.2d:%.2d:%.2d epoch=%ld : [%s] :%s\n",\
           the_date.tm_mday, the_date.tm_mon + 1, 1900 + the_date.tm_year, \
           the_date.tm_hour, the_date.tm_min, the_date.tm_sec, tm,  \
            __FUNCTION__, msg ); \
}


char * LOG_FILE_PATH = "/tmp/dse.log";

// msg should ne a null terminated string
static inline void log_vs_file(char *msg)
{
    time_t tm = time(NULL);
    struct tm the_date;
    localtime_r(&tm, &the_date);

    if(!log_vs_fd)
        log_vs_fd=open(LOG_FILE_PATH, O_RDWR, O_CREAT|O_DIRECT|O_APPEND);

    int n = snprintf(log_vs_buf, PIPE_BUF , "%.2d/%.2d/%.4d %.2d:%.2d:%.2d epoch=%ld : [%s] :%s\n",
                     the_date.tm_mday, the_date.tm_mon + 1, 1900 + the_date.tm_year,
                     the_date.tm_hour, the_date.tm_min, the_date.tm_sec, tm,
                     __FUNCTION__, msg );

    write(log_vs_fd, log_vs_buf, n );

}



#endif
