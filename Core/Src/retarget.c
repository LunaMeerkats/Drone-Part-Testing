// retarget.c — UART printf wiring only
#include "main.h"
#include <stdint.h>
#include <sys/unistd.h>   // STDOUT_FILENO, STDERR_FILENO
#include <stdio.h>        // FILE, setvbuf, stdout/stderr
#include <errno.h>

static UART_HandleTypeDef *s_huart = NULL;

void retarget_init(UART_HandleTypeDef *huart) {
    s_huart = huart;
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

static void uart_tx_blocking(const uint8_t *buf, uint16_t len) {
    if (!s_huart) return;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t ch = buf[i];
        if (ch == '\n') {
            const uint8_t crlf[2] = {'\r','\n'};
            HAL_UART_Transmit(s_huart, (uint8_t*)crlf, 2, HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(s_huart, &ch, 1, HAL_MAX_DELAY);
        }
    }
}

// Provide ONLY _write here. Let syscalls.c own _close/_fstat/_isatty/_lseek/_read.
int _write(int file, char *ptr, int len) {
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        uart_tx_blocking((uint8_t*)ptr, (uint16_t)len);
        return len;
    }
    errno = EBADF;
    return -1;
}

// Some libs route putchar -> fputc -> __io_putchar
int __io_putchar(int ch) {
    uint8_t c = (uint8_t)ch;
    uart_tx_blocking(&c, 1);
    return ch;
}

int fputc(int ch, FILE *f) {
    (void)f;
    return __io_putchar(ch);
}
