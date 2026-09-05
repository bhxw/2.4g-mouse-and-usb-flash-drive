#ifndef __CONSOLE_H
#define __CONSOLE_H

/**
 * @file console.h
 * @brief UART1 调试打印（printf 风格，无需 C 库重定向）
 */
void dbg_printf(const char *fmt, ...);

#endif /* __CONSOLE_H */
