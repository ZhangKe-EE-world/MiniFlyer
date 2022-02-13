#ifndef __SBUS_H
#define __SBUS_H
#include "sys.h"
#include <stdbool.h>

extern uint16_t CH[18];
extern uint8_t USART2_RX_BUF[26];

void SBUSInit(void);
void SBUS_Configuration(void);
void Sbus_Data_Count(uint8_t *buf);

#endif
