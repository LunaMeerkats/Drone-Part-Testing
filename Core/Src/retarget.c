#include "main.h"            // brings in HAL
#include <sys/unistd.h>      // _write
#include <stdint.h>

extern UART_HandleTypeDef huart3;  // <-- change to your actual UART handle (huart1/3/…)

/* Redirect printf to UART */
// printf -> UART3 (so you can use printf anywhere)
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
	return len;
}
