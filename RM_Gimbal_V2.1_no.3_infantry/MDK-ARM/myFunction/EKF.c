#include "EKF.h"

#define init_k_ 00000

#define GRAVITY 9.8f

#define offset_x 0
#define offset_y 46
#define offset_z 101
#define offset_pitch 0
#define offset_yaw 0
//#define kalman_k 20
float kalman_k = 300;

kalman_filter_init_t kalman_filter_init_data;
kalman_filter_t kalman_filter_auto_aim;
kalman_predict_t kalman_predict_auto_aim;

//×ø±êÏµ
        /*
        *      z: direction of the shooter
        *     /
        *  O /______x
        *    |
        *    |
        *    y
        */

float BulletModel(float z, float v, float angle) { //z:m,v:m/s,angle:rad
	float t, h;
	t = (float)((exp(init_k_ * z) - 1) / (init_k_ * v * cos(angle)));
	h = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
	return h;
}

float GetPitch(float z, float y, float v) //z:distance
{
	int i;
	float h_temp, h_actual, dh;
	float a;
	h_temp = -y;
	// by iteration
	for (i = 0; i < 20; i++)
	{
		a = (float) atan2(h_temp, z);
		h_actual = BulletModel(z, v, a);
		dh = -y - h_actual;
		h_temp = h_temp + dh;
		if (fabsf(dh) < 0.001)
			break;
	}
	return a;
}

void Transform(float xyz[],float v, float *pitch, float *yaw)
{
	xyz[0] += xyz[4] * kalman_k;
	xyz[1] += xyz[5] * kalman_k;
	xyz[2] += xyz[6] * kalman_k;
	
  if(init_k_!=0) *pitch = GetPitch((float)(xyz[2] + offset_z) / 100, -(float)(xyz[1] + offset_y) / 100, v)*180/PI + (float)offset_pitch;
  else {*pitch = -(atan2(xyz[1] + offset_y, xyz[2] + offset_z))*180/ PI;}

  //yaw positive direction :anticlockwise
  *yaw = atan2(xyz[0] + offset_x, xyz[2] + offset_z)*180 / PI + (float)offset_yaw;
}

void getPoint3f(float pitch,float yaw,float iuput_point[],float output_point[])
{
	
	
	float pitch_temp, yaw_temp;
	pitch_temp = pitch * PI / 180.0f;
	yaw_temp = -1* yaw * PI / 180.0f;
	mat xyz_input,xyz_output,ychange,xchange,TEMP;
	float xyz_inputdata[3]={0};
	float xyz_outputdata[3]={0,0,0};
	float TEMPdata[3]={0};
	float ychangedata[3][3]={
		0,0,0,
		0,1,0,
		0,0,0
	};

	float xchangedata[3][3]={
		1,0,0,
		0,0,0,
		0,0,0
	};	
	
	xyz_inputdata[0]=iuput_point[0];
	xyz_inputdata[1]=iuput_point[1]-46;
	xyz_inputdata[2]=iuput_point[2]+101;

	ychangedata[0][0]=arm_cos_f32(yaw_temp);												
	ychangedata[0][2]=arm_sin_f32(yaw_temp);												
	ychangedata[2][0]=-arm_sin_f32(yaw_temp);												
	ychangedata[2][2]=arm_cos_f32(yaw_temp);

	xchangedata[1][1]=arm_cos_f32(pitch_temp);
	xchangedata[1][2]=-arm_sin_f32(pitch_temp);
	xchangedata[2][1]=arm_sin_f32(pitch_temp);
	xchangedata[2][2]=arm_cos_f32(pitch_temp);													
																			
	mat_init(&xyz_input,3,1,(float *)xyz_inputdata);
	mat_init(&xyz_output,3,1,(float *)xyz_outputdata);
	mat_init(&TEMP,3,1,(float *)TEMPdata);
    //mat xyz_input(3,1,CV_32FC1,xyzdata);

    //cout<<xyz<<endl;
	mat_init(&ychange,3,3,(float *)ychangedata[0]);
	mat_init(&xchange,3,3,(float *)xchangedata[0]);

	mat_mult(&xchange, &xyz_input,&TEMP);
	mat_mult(&ychange, &TEMP,&xyz_output);

	output_point[0]=xyz_output.pData[0];
	output_point[1]=xyz_output.pData[1];
	output_point[2]=xyz_output.pData[2];									 
        //Mat r=yc*xc*xyz;
}



void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	
	I->xhat_data[0]=1;
	I->xhat_data[1]=1;
	I->xhat_data[2]=1;
	I->xhat_data[3]=1;
	I->xhat_data[4]=1;
	I->xhat_data[5]=1;


	I->P_data[0]=1;
	I->P_data[1]=0;	
	I->P_data[2]=0;
	I->P_data[3]=1;
	I->P_data[4]=1;
	I->P_data[5]=0;	
	
	I->P_data[6]=0;
	I->P_data[7]=1;
	I->P_data[8]=1;
	I->P_data[9]=0;	
	I->P_data[10]=0;
	I->P_data[11]=1;
	
	I->P_data[12]=1;
	I->P_data[13]=0;	
	I->P_data[14]=0;
	I->P_data[15]=1;
	I->P_data[16]=1;
	I->P_data[17]=0;
	
	I->P_data[18]=0;
	I->P_data[19]=1;
	I->P_data[20]=1;
	I->P_data[21]=0;	
	I->P_data[22]=0;
	I->P_data[23]=1;
	
	I->P_data[24]=1;
	I->P_data[25]=0;	
	I->P_data[26]=0;
	I->P_data[27]=1;
	I->P_data[28]=1;
	I->P_data[29]=0;
	
	I->P_data[30]=0;
	I->P_data[31]=1;
	I->P_data[32]=1;
	I->P_data[33]=0;	
	I->P_data[34]=0;
	I->P_data[35]=1;
	
	
	

	I->Q_data[0]=0.000000314;
	I->Q_data[1]=0;
	I->Q_data[2]=0;
	I->Q_data[3]=0;
	I->Q_data[4]=0;
	I->Q_data[5]=0;
	
	I->Q_data[6]=0;
	I->Q_data[7]=0.000000314;
	I->Q_data[8]=0;
	I->Q_data[9]=0;
	I->Q_data[10]=0;
	I->Q_data[11]=0;
	
	I->Q_data[12]=0;
	I->Q_data[13]=0;
	I->Q_data[14]=0.000000314;
	I->Q_data[15]=0;
	I->Q_data[16]=0;
	I->Q_data[17]=0;
	
	I->Q_data[18]=0;
	I->Q_data[29]=0;
	I->Q_data[20]=0;
	I->Q_data[21]=0.000000114;
	I->Q_data[22]=0;
	I->Q_data[23]=0;
	
	I->Q_data[24]=0;
	I->Q_data[25]=0;
	I->Q_data[26]=0;
	I->Q_data[27]=0;
	I->Q_data[28]=0.000000114;
	I->Q_data[29]=0;
	
	I->Q_data[30]=0;
	I->Q_data[31]=0;
	I->Q_data[32]=0;
	I->Q_data[32]=0;
	I->Q_data[34]=0;
	I->Q_data[35]=0.000000114;
	/////////////////////////////////////////////



	I->R_data[0]=1.8;
	I->R_data[1]=0;
	I->R_data[2]=0;
	I->R_data[3]=0;
	I->R_data[4]=0;
	I->R_data[5]=0;
	
	I->R_data[6]=0;
	I->R_data[7]=1.8;
	I->R_data[8]=0;
	I->R_data[9]=0;
	I->R_data[10]=0;
	I->R_data[11]=0;
	
	I->R_data[12]=0;
	I->R_data[13]=0;
	I->R_data[14]=1.8;
	I->R_data[15]=0;
	I->R_data[16]=0;
	I->R_data[17]=0;
	
	I->R_data[18]=0;
	I->R_data[29]=0;
	I->R_data[20]=0;
	I->R_data[21]=1.8;
	I->R_data[22]=0;
	I->R_data[23]=0;
	
	I->R_data[24]=0;
	I->R_data[25]=0;
	I->R_data[26]=0;
	I->R_data[27]=0;
	I->R_data[28]=1.8;
	I->R_data[29]=0;
	
	I->R_data[30]=0;
	I->R_data[31]=0;
	I->R_data[32]=0;
	I->R_data[32]=0;
	I->R_data[34]=0;
	I->R_data[35]=1.8;
	
	
/////////////////
	I->A_data[0]=1;
	I->A_data[1]=0;
	I->A_data[2]=0;
	I->A_data[3]=1;
	I->A_data[4]=0;
	I->A_data[5]=0;
	
	I->A_data[6]=0;
	I->A_data[7]=1;
	I->A_data[8]=0;
	I->A_data[9]=0;
	I->A_data[10]=1;
	I->A_data[11]=0;
	
	I->A_data[12]=0;
	I->A_data[13]=0;
	I->A_data[14]=1;
	I->A_data[15]=0;
	I->A_data[16]=0;
	I->A_data[17]=1;
	
	I->A_data[18]=0;
	I->A_data[29]=0;
	I->A_data[20]=0;
	I->A_data[21]=1;
	I->A_data[22]=0;
	I->A_data[23]=0;
	
	I->A_data[24]=0;
	I->A_data[25]=0;
	I->A_data[26]=0;
	I->A_data[27]=0;
	I->A_data[28]=1;
	I->A_data[29]=0;
	
	I->A_data[30]=0;
	I->A_data[31]=0;
	I->A_data[32]=0;
	I->A_data[32]=0;
	I->A_data[34]=0;
	I->A_data[35]=1;


///////////////////////
	I->H_data[0]=1;
	I->H_data[1]=0;
	I->H_data[2]=0;
	I->H_data[3]=0;
	I->H_data[4]=0;
	I->H_data[5]=0;
	
	I->H_data[6]=0;
	I->H_data[7]=1;
	I->H_data[8]=0;
	I->H_data[9]=0;
	I->H_data[10]=0;
	I->H_data[11]=0;
	
	I->H_data[12]=0;
	I->H_data[13]=0;
	I->H_data[14]=1;
	I->H_data[15]=0;
	I->H_data[16]=0;
	I->H_data[17]=0;
	
	I->H_data[18]=0;
	I->H_data[29]=0;
	I->H_data[20]=0;
	I->H_data[21]=1;
	I->H_data[22]=0;
	I->H_data[23]=0;
	
	I->H_data[24]=0;
	I->H_data[25]=0;
	I->H_data[26]=0;
	I->H_data[27]=0;
	I->H_data[28]=1;
	I->H_data[29]=0;
	
	I->H_data[30]=0;
	I->H_data[31]=0;
	I->H_data[32]=0;
	I->H_data[32]=0;
	I->H_data[34]=0;
	I->H_data[35]=1;


	
	/////////////////////////////////////////
	mat_init(&F->xhat,6,1,(float *)I->xhat_data);//must     start with zero usually
	mat_init(&F->xhatminus,6,1,(float *)I->xhatminus_data);

	mat_init(&F->z,6,1,(float *)I->z_data);	

	mat_init(&F->Pminus,6,6,(float *)I->Pminus_data);	
	mat_init(&F->P,6,6,(float *)I->P_data);//must

	mat_init(&F->K,6,6,(float *)I->K_data);

	mat_init(&F->Q,6,6,(float *)I->Q_data);//must
	mat_init(&F->R,6,6,(float *)I->R_data);//must

	mat_init(&F->A,6,6,(float *)I->A_data);//must
	mat_init(&F->H,6,6,(float *)I->H_data);//must


	mat_init(&F->AT,6,6,(float *)I->AT_data);
	mat_trans(&F->A, &F->AT);

	mat_init(&F->HT,6,6,(float *)I->HT_data);
	mat_trans(&F->H, &F->HT);
}


float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3, float signal4, float signal5, float signal6)//x,y,z,dx,dy,dz
{
	float TEMP_data[36] = {0, 0, 0, 0};
	float TEMP_data21[6] = {0, 0};
	mat TEMP,TEMP21;

	mat_init(&TEMP,6,6,(float *)TEMP_data);
	mat_init(&TEMP21,6,1,(float *)TEMP_data21);

	F->z.pData[0] = signal1;
	F->z.pData[1] = signal2;
	F->z.pData[2] = signal3;
	F->z.pData[3] = signal4;
	F->z.pData[4] = signal5;
	F->z.pData[5] = signal6;

	//1. xhat'(k)= A xhat(k-1)
	mat_mult(&F->A, &F->xhat, &F->xhatminus);

	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&F->A, &F->P, &F->Pminus);
	mat_mult(&F->Pminus, &F->AT, &TEMP);
	mat_add(&TEMP, &F->Q, &F->Pminus);

	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&F->H, &F->Pminus, &F->K);
	mat_mult(&F->K, &F->HT, &TEMP);
	mat_add(&TEMP, &F->R, &F->K);

	mat_inv(&F->K, &F->P);
	mat_mult(&F->Pminus, &F->HT, &TEMP);
	mat_mult(&TEMP, &F->P, &F->K);

	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&F->H, &F->xhatminus, &TEMP21);
	mat_sub(&F->z, &TEMP21, &F->xhat);
	mat_mult(&F->K, &F->xhat, &TEMP21);
	mat_add(&F->xhatminus, &TEMP21, &F->xhat);

	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&F->K, &F->H, &F->P);
	mat_mult(&F->P, &F->Pminus, &TEMP);
	mat_sub(&F->Pminus, &TEMP, &F->P);

	F->filtered_value[0] = F->xhat.pData[0];
	F->filtered_value[1] = F->xhat.pData[1];
	F->filtered_value[2] = F->xhat.pData[2];
	F->filtered_value[3] = F->xhat.pData[3];
	F->filtered_value[4] = F->xhat.pData[4];
	F->filtered_value[5] = F->xhat.pData[5];

	return F->filtered_value;
}


