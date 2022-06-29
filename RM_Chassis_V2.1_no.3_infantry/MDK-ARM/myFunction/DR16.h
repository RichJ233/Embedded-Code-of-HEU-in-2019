#ifndef __DR16_H_
#define __DR16_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

/*user's incluide*/

/////////////////////

#define key_w     ((uint16_t)0x01<<0)
#define key_s     ((uint16_t)0x01<<1)
#define key_a     ((uint16_t)0x01<<2)
#define key_d     ((uint16_t)0x01<<3)
#define key_q 		((uint16_t)0x01<<6)
#define key_e 		((uint16_t)0x01<<7)
#define key_shift ((uint16_t)0x01<<4)
#define key_ctrl 	((uint16_t)0x01<<5)
#define key_r     ((uint16_t)0x01<<8)
#define key_f     ((uint16_t)0x01<<9)
#define key_g 		((uint16_t)0x01<<10)
#define key_z  		((uint16_t)0x01<<11)
#define key_x     ((uint16_t)0x01<<12)
#define key_c     ((uint16_t)0x01<<13)
#define key_v 		((uint16_t)0x01<<14)
#define key_b  		((uint16_t)0x01<<15)

typedef struct{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	float channel[4];
	uint8_t sl;
	uint8_t sr;
}rc_t;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
}mouse_t;

typedef struct{
	uint16_t val;
	uint16_t w;
	uint16_t s;
	uint16_t a;
	uint16_t d;
	uint16_t shift;
	uint16_t ctrl;
	uint16_t q;
	uint16_t e;
	uint16_t r;
	uint16_t f;
	uint16_t g;
	uint16_t z;
	uint16_t x;
	uint16_t c;
	uint16_t v;
	uint16_t b;
}key_t;

typedef struct { 
	rc_t rc;
	mouse_t mouse;
	key_t key;
	float mouse_x;
	float mouse_y;
}RC_Ctl_t;


void DR16_decode(uint8_t *Rx,RC_Ctl_t *decode);
void KeyBoard_decode(RC_Ctl_t *decode);

extern RC_Ctl_t RC_Ctl;
extern unsigned char RCrx[50];

#endif



/*
¼üÅÌ
Bit0------W     1
Bit1------S     2
Bit2------A     4
Bit3------D     8
Bit4------Shift 10
Bit5------Ctrl  20
Bit6------Q     40 
Bit7------E     80
Bit8------R     100
Bit9------F     200
Bit10-----G     400
Bit11-----Z     800
Bit12-----X     1000
Bit13-----C     2000
Bit14-----V     4000
Bit15-----B     8000
*/



