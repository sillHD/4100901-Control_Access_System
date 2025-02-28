#ifndef __KEYPAD_H
#define __KEYPAD_H

#include <stdint.h>

void keypad_init(void);
uint8_t keypad_scan(uint16_t GPIO_Pin);

#endif // __KEYPAD_H