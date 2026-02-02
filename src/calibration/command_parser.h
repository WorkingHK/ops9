#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include "types.h"

// Command callback function type
typedef void (*command_callback_t)(const char *args);

// Initialize command parser
void command_parser_init(void);

// Process incoming command string
void command_parser_process(const char *cmd_string);

// Register calibration context
void command_parser_set_context(imu_state_t *imu_state, calibration_t *calibration);

#endif // COMMAND_PARSER_H
