/*
 * test_practice.c
 *
 *  Created on: 11 paź 2021
 *      Author: macie
 */




#define TEST	0x40021018U
#include "stm32f103rb.h"

void dupka(void)
{

	volatile unsigned int *p;
	p = TEST;
	*p |= 1<<2;

	int Z = &(RCC->APB2ENR);
}

void dupka2(void)
{
//	long b;
//	b  <--- wartość b
//	&b <--- adres b
//
//	long* c;
//	c  <--- adres tego, na co wskazuje c
//	&c <--- adres c
//	*c <--- operator wyłuskania (wartość tego na co wskazuje c)
//
//
//
//
//	void ZamienWartosci(long* a, long* b) {
//		long tmp = *b;
//		*b = *a;
//		*a = tmp;
//	}
//
//	def zmien_wartosci(x, y):
//			return y, x
//
//	a, b = zmien_wartosci(a, b)

}
