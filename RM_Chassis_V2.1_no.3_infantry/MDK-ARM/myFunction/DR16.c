/**
  ******************************************************************************
	*DATE			: 2018/3/1
	*AUTHOR		:	RichJ
	*VERSION	:	V1.0
	*BRIEF		: Complete the decode function;Test success;
  ******************************************************************************
  */
	
#include "DR16.h"
#include "bsp_can.h"
#include "RM_Define.h"

unsigned char RCrx[50];
int position_X=0,position_Y=0,K=0;
/* user decode struct define begin */
RC_Ctl_t RC_Ctl;
uint8_t start_flag = 0;
/* user decode struct define end */


void DR16_decode(uint8_t *Rx,RC_Ctl_t *decode)
{
	decode->rc.ch0 = (Rx[0]| (Rx[1] << 8)) & 0x07ff; 															//!< Channel 0
	decode->rc.ch1 = ((Rx[1] >> 3) | (Rx[2] << 5)) & 0x07ff; 											//!< Channel 1
	decode->rc.ch2 = ((Rx[2] >> 6) | (Rx[3] << 2) | (Rx[4] << 10)) & 0x07ff;			//!< Channel 2
	decode->rc.ch3 = ((Rx[4] >> 1) | (Rx[5] << 7)) & 0x07ff; 											//!< Channel 3
	decode->rc.sl = ((Rx[5] >> 4)& 0x000C) >> 2; 																	//!< Switch left
	decode->rc.sr = ((Rx[5] >> 4)& 0x0003);  																			//!< Switch right
	decode->mouse.x = Rx[6] | (Rx[7]  << 8);						 												  //!< Mouse X axis
	decode->mouse.y = Rx[8] | (Rx[9]  << 8); 																			//!< Mouse Y axis
	decode->mouse.z = Rx[10] | (Rx[11] << 8); 																		//!< Mouse Z axis
	decode->mouse.press_l = Rx[12];																							  //!< Mouse Left Is Press Down
	decode->mouse.press_r = Rx[13];																							  //!< Mouse Right Is Press Down
	decode->key.val = Rx[14] | (Rx[15]  << 8); 																		//!< KeyBoard Value
	KeyBoard_decode(decode);																							  						//!< KetBoard Decode
	position_X=decode->mouse.x*K+position_X; 																			//!< Mouse X Position
	position_Y=decode->mouse.y*K+position_Y; 																			//!< Mouse Y Position
	
	decode->rc.channel[0] = (decode->rc.ch0 - 1024) * Channel_K0;
	decode->rc.channel[1] = (decode->rc.ch1 - 1024) * Channel_K1;
	decode->rc.channel[2] = (decode->rc.ch2 - 1024) * Channel_K2;	
	decode->rc.channel[3] = (decode->rc.ch3 - 1024) * Channel_K3;
	
	decode->mouse_x = decode->mouse.x * Mouse_X_K;
	decode->mouse_y = decode->mouse.y * Mouse_Y_K;
}

void KeyBoard_decode(RC_Ctl_t *decode)
{
	decode->key.w = (decode->key.val & key_w) >> 0;
	decode->key.s = (decode->key.val & key_s) >> 1;
	decode->key.a = (decode->key.val & key_a) >> 2;
	decode->key.d = (decode->key.val & key_d) >> 3;
	decode->key.shift = (decode->key.val & key_shift) >> 4;
	decode->key.ctrl = (decode->key.val & key_ctrl) >> 5;
	decode->key.q = (decode->key.val & key_q) >> 6;
	decode->key.e = (decode->key.val & key_e) >> 7;
	decode->key.r = (decode->key.val & key_r) >> 8;
	decode->key.f = (decode->key.val & key_f) >> 9;
	decode->key.g = (decode->key.val & key_g) >> 10;
	decode->key.z = (decode->key.val & key_z) >> 11;
	decode->key.x = (decode->key.val & key_x) >> 12;
	decode->key.c = (decode->key.val & key_c) >> 13;
	decode->key.v = (decode->key.val & key_v) >> 14;
	decode->key.b = (decode->key.val & key_b) >> 15;
}

float RC_limit(float Output,float Min,float Max)
{
    if(Output>Max)
        Output = Max;
    else
        if(Output<Min)
            Output = Min;
    return Output;
}

