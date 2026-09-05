#ifndef __SD_PROBE_H
#define __SD_PROBE_H

/**
 * @file sd_probe.h
 * @brief SD/FatFs 自检任务：挂载 FAT32 → 写/读 1 扇区回环校验 → UART 输出结果
 */
void sd_probe_start(void);

#endif /* __SD_PROBE_H */
