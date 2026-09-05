#ifndef __SD_LOG_H
#define __SD_LOG_H

#include "nrf_demo.h"   /* MousePacket_t */

/**
 * @file sd_log.h
 * @brief SD 日志链路（M2c）：rf 任务投递数据包 → 队列 → sd_log 任务批量写 DATA.LOG
 */
void sd_log_start(void);
void sd_log_write_packet(const MousePacket_t *p);

#endif /* __SD_LOG_H */
