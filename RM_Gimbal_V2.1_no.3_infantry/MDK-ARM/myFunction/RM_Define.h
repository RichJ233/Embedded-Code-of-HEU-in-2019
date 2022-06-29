#ifndef __RM_DEFINE_H__
#define __RM_DEFINE_H__

#define gimbal_test 0

//��̨��PIT��Ҫ���Ĳ����������޷��Լ���ֵ�������ƺ�����Ƿ�װ��С�������棬��������档
#define PIT_8192 1					//������̨PIT����Ƿ񳬹�����8192��������Ϊ1����֮��
#define PITCH_MID 9520
//#define PITCH_UP 8500
#define PITCH_UP -0.128784f
//#define PITCH_DOWN 9960
#define PITCH_DOWN 0.049438f

#define YAW_8192 1					//������̨YAW����Ƿ񳬹�����8192��������Ϊ1����֮��
#define YAW_MID 6114

//gimbal pid param tune
#define GIMBAL_PID_TEST 0

//���ֶ��ֵ
#define Open_CCR 1200
#define Close_CCR 1730

typedef enum
{
  SILENCE_MODE    = 0x00,
  CONTROL_MODE    = 0x01,
  AUTO_AIM_MODE   = 0x02,
  AUTO_AIM_MODE_NO= 0x03,
  AUTO_GAIN_MODE  = 0x04,
  RESET_MODE      = 0xFF,
} gim_mod_id_e;

#define BULLET_SPEED 20

#endif

