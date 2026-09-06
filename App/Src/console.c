/**
 * @file console.c
 * @brief UART1 调试打印：vsnprintf 后整包发送（不依赖库重定向）
 */
#include "console.h"

#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void dbg_printf(const char *fmt, ...)
{
    char buf[128];
    int n;
    va_list ap;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return;
    }
    if (n > (int)sizeof(buf))
    {
        n = (int)sizeof(buf);
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
}

void dbg_puts(const char *s)
{
    uint16_t n = (uint16_t)strlen(s);

    while (n > 0U)
    {
        uint16_t chunk = (n > 128U) ? 128U : n;
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, chunk, HAL_MAX_DELAY);
        s += chunk;
        n = (uint16_t)(n - chunk);
    }
}
