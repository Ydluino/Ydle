/*
 * Float.h
 *
 * Created on: Feb 8, 2014
 * Author: denia
 */

#ifndef FLOAT_H
#define FLOAT_H

#include <stdint.h>

typedef uint16_t float16 ;

typedef union _FP32 {
	uint32_t u;
	float f;
	struct {
		uint32_t Mantissa : 23;
		uint32_t Exponent : 8;
		uint32_t Sign : 1;
	};
} uFP32_t ;

typedef union _FP16 {
	float16 f16;
	struct {
		uint16_t Mantissa : 10;
		uint16_t Exponent : 5;
		uint16_t Sign : 1;
	};
} uFP16_t ;

uFP32_t half_to_float_full(uFP16_t h) ;
uFP16_t float_to_half_full(uFP32_t f) ;

extern float16 Float32To16 (float) ;
extern float Float16To32 (float16) ;

#endif // FLOAT_H