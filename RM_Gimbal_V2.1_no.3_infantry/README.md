/*
 ******************************************************************************
 *Author:		Rich_J
 *This document complies with the MIT Convention
 *This document is partially written in accordance with Huawei's programming specifications, 
 *and is partially written in accordance with the DJI specification.
 ******************************************************************************
 */
 
/********************************************************/
Version:	V1.0_Release
Date:		19/6/2019
Brief: 		1 Complete the basic functions, see RM_Define.h for specific interface parameters.
			2 Unadjusted PID parameters.
			3 Unadjusted PTZ servo value (not tested).
/********************************************************/

/********************************************************/
Version:	V1.1_Alpha
Date:		25/6/2019
Brief: 		1 BUG Solved:TIM prescaler changed from 84 to 83(83-1).
			2 Adjusted the PID param of PTZ(pid_yaw_speed,pid_yaw_position,pid_pit_speed,pid_pit_position).
			2 Solve the gyro infantry's imu angle can't calculate more than 360 degree problem.
			3 Test the code to RESET the mcu:			__set_FAULTMASK(1);
														NVIC_SystemReset();
			4 Unsloved when mcu RESET, imu should wait a long time to get offset.
			5 Next version can use flash to hook offset and RESET param.
/********************************************************/

/********************************************************/
Version:	V1.1_Beta
Date:		29/6/2019
Brief: 		1 ADD AUTO_GAIN_MODE and it's code.
			2 By using flash to save param to skip gyro offset process.
			3 Next version should test on the infantry.
/********************************************************/