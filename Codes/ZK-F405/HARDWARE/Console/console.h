#ifndef __CONSOLE_H
#define __CONSOLE_H
#include <stdbool.h>
#include "sys.h"



void consoleInit(void);
bool consoleTest(void);
int consolePutchar(int ch);	/* 输入一个字符到console缓冲区*/
int consolePutcharFromISR(int ch);	/* 中断方式输入一个字符到console缓冲区*/
int consolePuts(char *str);	/* 输入一个字符串到console缓冲区*/

#endif /*__CONSOLE_H*/
