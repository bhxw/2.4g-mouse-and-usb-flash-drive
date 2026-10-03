#ifndef __CONSOLE_H
#define __CONSOLE_H

#include <stdint.h>

/**
 * @file console.h
 * @brief UART1 调试打印（printf 风格，无需 C 库重定向）
 */
void dbg_printf(const char *fmt, ...);

/* 整段发送（用于 vTaskList / vTaskGetRunTimeStats 等大块文本） */
void dbg_puts(const char *s);

/* 累计丢包次数：HAL_UART_Transmit 在 gState != READY 时立刻返回 HAL_BUSY
 * 且一个字节都不发。多任务共用 UART1 时这是静默丢行，计数后才能区分
 * "根本没打" 和 "打了但被丢"。 */
uint32_t dbg_console_drops(void);

#endif /* __CONSOLE_H */
