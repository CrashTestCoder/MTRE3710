/*
 * GccApplication1.c
 *
 * Created: 10/8/2019 1:03:18 PM
 * Author : mrrya
 */ 

#include <avr/io.h>

#ifndef _7_SEG_H_
#define _7_SEG_H_

#include "avr/io.h"

typedef enum {
    _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _a, _b, _c, _d, _e, _f
} _7_seg_vals : uint8_t;

/**
 * ____a____
 * |       |
 * |f      |b
 * |___g___|
 * |       |
 * |e      |c
 * |___d___|  dp
 */

typedef enum {
	dp = 0b10000000,
	a  = 0b01000000,
	b  = 0b00100000,
	c  = 0b00010000,
	d  = 0b00001000,
	e  = 0b00000100,
	f  = 0b00000010,
	g  = 0b00000001,
} _7_seg_map : uint8_t;

uint8_t const num_map[] = {
    [_0] = a|b|c|d|e|f,
    [_1] =   b|c,
    [_2] = a|b|c|d|e|  g,
    [_3] = a|b|c|d|    g,
    [_4] =   b|c|    f|g,
    [_5] = a|  c|d|  f|g,
    [_6] = a|  c|d|e|f|g,
    [_7] = a|b|c,
    [_8] = a|b|c|d|e|f|g,
    [_9] = a|b|c|    f|g,
    [_a] = a|b|c|  e|f|g,
    [_b] =     c|d|e|f|g,
    [_c] =       d|e|  g,
    [_d] =   b|c|d|e|  g,
    [_e] = a|    d|e|f|g,
    [_f] = a|      e|f|g
};

void disp(_7_seg_vals val)
{
    PORTD = num_map[val];
}

#endif /* _7_SEG_H_ */
