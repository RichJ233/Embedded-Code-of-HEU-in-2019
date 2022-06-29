#ifndef __RM_DEFINE_H__
#define __RM_DEFINE_H__

#define gimbal_test 0

//云台就PIT需要更改参数，上下限幅以及中值。其中似乎电机是反装的小的在上面，大的在下面。
#define PIT_8192 1					//用于云台PIT电机是否超过量程8192，超过则为1，反之。
#define PITCH_MID 9520
//#define PITCH_UP 8500
#define PITCH_UP -0.128784f
//#define PITCH_DOWN 9960
#define PITCH_DOWN 0.049438f

#define YAW_8192 1					//用于云台YAW电机是否超过量程8192，超过则为1，反之。
#define YAW_MID 6114

//gimbal pid param tune
#define GIMBAL_PID_TEST 0

//弹仓舵机值
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

