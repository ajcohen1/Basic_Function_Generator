/*
 * KEYPAD_DRIVER.c
 *
 *  Created on: Apr 3, 2020
 *      Author: ajcoh
 */


#include "KEYPAD_DRIVER.h"

void keypad_init(void) {
    P4->DIR = 0;
    P4->REN |= (ROW1 | ROW2 | ROW3 | ROW4);
    P4->OUT &= ~(ROW1 | ROW2 | ROW3 | ROW4);
}

uint8_t keypad_getkey(void) {
    uint8_t row = 0, col = 0, key = 0;

    P4->DIR |= (COL1 | COL2 | COL3);
    P4->OUT |= (COL1 | COL2 | COL3);
    _delay_cycles(25);

    row = P4->IN & (ROW1 | ROW2 | ROW3 | ROW4);

    if(row == 0)
        return 0xFF;
    for (col = 0; col < 3; col++) {

        P4->OUT &= ~(COL1 | COL2 | COL3);

        P4->OUT |= (COL1 << col);
        _delay_cycles(25);

        row = P4->IN & (ROW1 | ROW2 | ROW3 | ROW4);

        if(row != 0) break;
    }

    P4->OUT &= ~(COL1 | COL2 | COL3);
    P4->DIR &= ~(COL1 | COL2 | COL3);

    if (col == 3) return 0xFF;

    if(row == 4) row = 3;
    if(row == 8) row = 4;

    if(col == 0) key = row*3 - 2;
    if(col == 1) key = row*3 - 1;
    if(col == 2) key = row*3;

    if(key == 11) key = 0;

    return key;
    }
