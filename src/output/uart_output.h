#ifndef UART_OUTPUT_H
#define UART_OUTPUT_H

#include "types.h"
#include "esp_err.h"

// Initialize and start UART output task
esp_err_t uart_output_start(position_state_t *position_state);

#endif // UART_OUTPUT_H
