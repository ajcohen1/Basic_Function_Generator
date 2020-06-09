/*
 * KEYPAD_DRIVER.h
 *
 *  Created on: Apr 3, 2020
 *      Author: ajcoh
 */

#ifndef KEYPAD_DRIVER_H_
#define KEYPAD_DRIVER_H_

#include "msp.h"
#include <stdio.h>
#include <stdint.h>

#define COL1 BIT4  //4.4
#define COL2 BIT5   //4.5
#define COL3 BIT6   //4.6
#define ROW1 BIT0  //4.0
#define ROW2 BIT1   //4.1
#define ROW3 BIT2   //4.2
#define ROW4 BIT3   //4.3

#define DEFAULT_KEY 255

void keypad_init(void);
uint8_t keypad_getkey(void);
uint8_t keypad_isUniqueKey(uint8_t key, uint8_t prior_key);
void delay(void);


#endif /* KEYPAD_DRIVER_H_ */
