/*
 * main.h
 *
 *  Created on: Jul 20, 2025
 *      Author: Nikhil Sagar
 */

#ifndef MAIN_H_
#define MAIN_H_



typedef struct{
	uint32_t GPIOAEN :1;
	uint32_t GPIOBEN :1;
	uint32_t GPIOCEN :1;
	uint32_t GPIODEN :1;
	uint32_t GPIOEEN :1;
	uint32_t GPIOFEN :1;
	uint32_t GPIOGEN :1;
	uint32_t GPIOHEN :1;
	uint32_t GPIOIEN :1;
	uint32_t Reserved1 :3;
	uint32_t CRCEN :1;
	uint32_t Reserved2 :5;
	uint32_t BKPSRAMEN :1;
	uint32_t Reserved3 :1;
	uint32_t CCMDATARAMEN:1;
	uint32_t DMA1EN :1;
	uint32_t DMA2EN :1;
	uint32_t Reserved4 :2;
	uint32_t ETHMACEN :1;
	uint32_t ETHMACTXEN :1;
	uint32_t ETHMACRXEN :1;
	uint32_t ETHMACPTPEN :1;
	uint32_t OTGHSEN :1;
	uint32_t OTGHSULPIEN :1;
	uint32_t Reserved5 :1;
}RCC_AHB1ENR;



typedef struct{
	uint32_t MODER0 :2;
	uint32_t MODER1 :2;
	uint32_t MODER2 :2;
	uint32_t MODER3 :2;
	uint32_t MODER4 :2;
	uint32_t MODER5 :2;
	uint32_t MODER6 :2;
	uint32_t MODER7 :2;
	uint32_t MODER8 :2;
	uint32_t MODER9 :2;
	uint32_t MODER10 :2;
	uint32_t MODER11 :2;
	uint32_t MODER12 :2;
	uint32_t MODER13 :2;
	uint32_t MODER14 :2;
	uint32_t MODER15 :2;
}GPIOx_MODER;

typedef struct{
	uint32_t IDR0 :1;
	uint32_t IDR2 :1;
	uint32_t IDR3 :1;
	uint32_t IDR4 :1;
	uint32_t IDR5:1;
	uint32_t IDR6 :1;
	uint32_t IDR7 :1;
	uint32_t IDR8 :1;
	uint32_t IDR9 :1;
	uint32_t IDR10 :1;
	uint32_t IDR11 :1;
	uint32_t IDR12 :1;
	uint32_t IDR13 :1;
	uint32_t IDR14 :1;
	uint32_t IDR15 :1;
	uint32_t Reserved :15;
}GPIOx_IDR;

typedef struct{
	uint32_t ODR0 :1;
	uint32_t ODR1 :1;
	uint32_t ODR2 :1;
	uint32_t ODR3 :1;
	uint32_t ODR4 :1;
	uint32_t ODR5:1;
	uint32_t ODR6 :1;
	uint32_t ODR7 :1;
	uint32_t ODR8 :1;
	uint32_t ODR9 :1;
	uint32_t ODR10 :1;
	uint32_t ODR11 :1;
	uint32_t ODR12 :1;
	uint32_t ODR13 :1;
	uint32_t ODR14 :1;
	uint32_t ODR15 :1;
	uint32_t Reserved :15;
}GPIOx_ODR;


#endif /* MAIN_H_ */
