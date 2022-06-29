#ifndef __EKF_H__
#define __EKF_H__

#include "stm32f4xx.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

typedef struct
{
  float raw_value;
  float filtered_value[6];
	
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
}kalman_filter_t;

typedef struct
{
	float xyz_last_data[3]; 
	float xyz_input_data[3]; 
	float xyz_output_data[3]; 
	float xyz_increase_data[3]; 
	float xyz_predict_data[6];
}kalman_predict_t;

typedef struct
{
  float raw_value;
  float filtered_value[6];
	
  float xhat_data[6], xhatminus_data[6], z_data[6],Pminus_data[36], K_data[36];// x'  predict x   measure   predict p    
  float P_data[36];
	
  float AT_data[36], HT_data[36];
	
  float A_data[36];
  float H_data[36];
  float Q_data[36];
  float R_data[36];
}kalman_filter_init_t;

extern kalman_filter_init_t kalman_filter_init_data;
extern kalman_filter_t kalman_filter_auto_aim;
extern float xyz_inputdata[3];
extern float xyz_outputdata[3];
extern kalman_predict_t kalman_predict_auto_aim;

void getPoint3f(float pitch,float yaw,float iuput_point[],float output_point[]);
void Transform(float xyz[],float v, float *pitch, float *yaw);
float GetPitch(float z, float y, float v);
float BulletModel(float z, float v, float angle);
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3, float signal4, float signal5, float signal6);//x,y,z,dx,dy,dz


#endif
