/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "bsp_rc.h"
#include "main.h"
#include "struct_typedef.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

#define K_Wheel 5
#define K_Pitch 0.0001
#define K_Yaw 0.000001

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
				short wheel;
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;
		uint32_t misstimeout;
		__packed struct
		{
			bool CTRL;
			bool F;
			bool X;
			bool C;
			bool Q;
			bool E;
			bool Z;
		}flag;//标志位，用来切换模式，保证按键瞬间只进入函数一次

} RC_ctrl_t;

typedef struct
{
	float front;
	float right;
	float rotate;
	float pitch;
	float yaw;
}Direc_Struct;

/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (rc_ctrl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (rc_ctrl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (rc_ctrl.mouse.z)


/* 检测鼠标按键状态
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc_ctrl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc_ctrl.mouse.press_r == 1)


/* 检测键盘按键状态
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  rc_ctrl.key.v  )
#define    IF_KEY_PRESSED_W       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#define    IF_KEY_PRESSED_LAST         (  rc_ctrl.key.v_last  )
#define    IF_KEY_PRESSED_W_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R_LAST       ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL_LAST    ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT_LAST   ( (rc_ctrl.key.v_last & KEY_PRESSED_OFFSET_SHIFT) != 0 )

/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
extern const RC_ctrl_t *get_remote_control_point(void);

void Trans_Control(Direc_Struct *Control, RC_ctrl_t *RC);
void Remote_Handler(void);

extern RC_ctrl_t rc_ctrl;
extern Direc_Struct direc;

#endif
