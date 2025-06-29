/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "can.h"
#include "BSP_CAN.h"
#include "stm32f4xx_hal_can.h"
//#include "cap.h"
#include "type.h"
#include "SuperPower.h"
moto_measure_t 		moto_CAN1[11] = {0};
moto_measure_t 		moto_CAN2[11] = {0};
C_Board_measure_t 	C_Board;
float cap_remain_percent;
uint32_t receive=0 ;

void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterTypeDef		sFilterConfig;


    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterBank=27;
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;
    if(HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
    {

    }


}


/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
uint8_t tempsee[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{
    CAN_RxHeaderTypeDef _RxHeader;
		uint8_t temp[16];
    HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&_RxHeader,temp);
	
    //ignore can1 or can2.
    switch(_RxHeader.StdId) {
    case CAN_Moto1_ID:
    case CAN_Moto2_ID:
    case CAN_Moto3_ID:
    case CAN_Moto4_ID:
    case CAN_Moto5_ID:
    case CAN_Moto6_ID:
    case CAN_Moto7_ID:
    case CAN_Moto8_ID:
    case CAN_Moto9_ID:
    case CAN_Moto10_ID:
    case CAN_Moto11_ID:
		{
			static u8 i;
			i = _RxHeader.StdId - CAN_Moto1_ID;
			if(_hcan==&hcan1)
			{		
				get_moto_measure(&moto_CAN1[i],temp);;
			}
			else
				get_moto_measure(&moto_CAN2[i], temp);

		}
		break;
		case Moto_Yaw_4310:
		{
			get_4310moto_measure(temp,&moto_CAN2[9]);
		}
		break;
		case Moto_Pitch_4310:
		{
			get_4310moto_measure(temp,&moto_CAN2[10]);
		}
		break;
		case C_Board_ID:
		{
			get_C_Board_measure(&C_Board,temp);
		}
		break;
		case 0x0C2:
		{
			SuperPower_Rx(temp);
		}
		break;

//	case CAN_SuperCap_ID:
//		{
//            cap_add++;
//            receive++;
//			get_cap_measure(&Cap_can_rxbuffer[8],temp);
//			tempsee[0]=temp[0];
//			tempsee[1]=temp[1];
//			tempsee[2]=temp[2];
//			tempsee[3]=temp[3];
//		 }
    }
    __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr,uint8_t temp[])
{
    //CAN_RX_data_add++;
	ptr->last_angle     =  ptr->angle;
	ptr->angle          = (uint16_t)(((uint16_t)temp[0]) << 8 | (uint16_t)temp[1]);
	ptr->speed_rpm      = (int16_t)((int16_t)temp[2] << 8 | (int16_t)temp[3]);
	ptr->real_current  = (int16_t)((int16_t)temp[4] << 8 | (int16_t)temp[5]);
	ptr->hall       =  (int16_t)temp[6];
	get_total_angle(ptr);
	ptr->RX_data.RX_add++;

}

void get_4310moto_measure(uint8_t temp[],moto_measure_t *ptr){
	
			ptr->p_int			=	(temp[1]<<8)|temp[2];
			ptr->v_int			=	(temp[3]<<4)|(temp[4]>>4);
			ptr->t_int			=	((temp[4]&0xF)<<8)|temp[5];
			ptr->position 	= uint_to_float(ptr->p_int, P_MIN, P_MAX, 16); // (-3.141593	,	3.141593)
			ptr->velocity 	= uint_to_float(ptr->v_int, V_MIN, V_MAX, 12); // (-30.0			,	30.0		)
			ptr->torque 		= uint_to_float(ptr->t_int, T_MIN, T_MAX, 12); // (-10.0			,	10.0		)
	

}

void get_C_Board_measure(C_Board_measure_t *ptr,uint8_t temp[])
{
			memcpy(ptr,temp,8);
	
			ptr->x 				= uint_to_float(ptr->x_int, X_MIN, X_MAX, 12)+0.000732660294; // (-3.000	,	3.000)
			ptr->y 				= uint_to_float(ptr->y_int, Y_MIN, Y_MAX, 12)+0.000732660294; // (-3.000	,	3.000)
			ptr->w 				= uint_to_float(ptr->w_int, W_MIN, W_MAX, 12)+0.0021982193; // (-9.000	,	9.000)
}


/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
    ptr->angle = 0;
    ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p) {

    int res1, res2, delta;
    if(p->angle < p->last_angle) {			//可能的情况
        res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    } else {	//angle > last
        res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(ABS(res1)<ABS(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
}

void set_moto1234_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp[8];
    _TxHeader.StdId = 0x200;
    _TxHeader.IDE = CAN_ID_STD;
    _TxHeader.RTR = CAN_RTR_DATA;
    _TxHeader.DLC = 0x08;
    Txtemp[0] = (iq1 >> 8);
    Txtemp[1] = iq1;
    Txtemp[2] = (iq2 >> 8);
    Txtemp[3] = iq2;
    Txtemp[4] = iq3 >> 8;
    Txtemp[5] = iq3;
    Txtemp[6] = iq4 >> 8;
    Txtemp[7] = iq4;
    while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 );
    if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
    {

    }
}

void set_moto5678_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp[8];
    _TxHeader.StdId = 0x1FF;
    _TxHeader.IDE = CAN_ID_STD;
    _TxHeader.RTR = CAN_RTR_DATA;
    _TxHeader.DLC = 0x08;
    Txtemp[0] = (iq1 >> 8);
    Txtemp[1] = iq1;
    Txtemp[2] = (iq2 >> 8);
    Txtemp[3] = iq2;
    Txtemp[4] = iq3 >> 8;
    Txtemp[5] = iq3;
    Txtemp[6] = iq4 >> 8;
    Txtemp[7] = iq4;
    while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0);
    if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
    {

    }

}

void set_moto91011_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3 ) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp[8];
    _TxHeader.StdId = 0x2FF;
    _TxHeader.IDE = CAN_ID_STD;
    _TxHeader.RTR = CAN_RTR_DATA;
    _TxHeader.DLC = 0x08;
    Txtemp[0] = (iq1 >> 8);
    Txtemp[1] = iq1;
    Txtemp[2] = (iq2 >> 8);
    Txtemp[3] = iq2;
    Txtemp[4] = iq3 >> 8;
    Txtemp[5] = iq3;
//	Txtemp[6] = iq4 >> 8;
//	Txtemp[7] = iq4;
    while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 );//改can
    if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
    {

    }

}
//4310

char Selection=0;

uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//电机保存零点命令


/**
 * @brief  这里对ID为0x02、0x03、0x04的3个电机进行依次使能，在对电机进行控制前进行使能
 * @param  void     	
 * @param  vodi      
 * @param  void    	
 * @param  void      	
 */
void Motor_enable(void)
{
	#if Motar_mode==0
	CANx_SendStdData(&hcan2,0x107,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan2,0x109,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan2,0x305,Data_Enable,8);
	HAL_Delay(10);
	#elif Motar_mode==1
	CANx_SendStdData(&hcan1,0x102,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x103,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x104,Data_Enable,8);
	HAL_Delay(10);
	#elif Motar_mode==2
	CANx_SendStdData(&hcan1,0x202,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x203,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x204,Data_Enable,8);
	HAL_Delay(10);
	#endif

}

uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
	
	
        /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
    return 0;
}

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)
 { 
	static CAN_TxHeaderTypeDef   Tx_Header;
	uint8_t Txtemp[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	Tx_Header.StdId=id;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=0x08;
	
	Txtemp[0] = (pos_tmp >> 8);
	Txtemp[1] = pos_tmp;
	Txtemp[2] = (vel_tmp >> 4);
	Txtemp[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	Txtemp[4] = kp_tmp;
	Txtemp[5] = (kd_tmp >> 4);
	Txtemp[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	Txtemp[7] = tor_tmp;
	 
	 //寻空邮箱发送数据
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, Txtemp, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, Txtemp, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, Txtemp, (uint32_t*)CAN_TX_MAILBOX2);
        }
   }
 }


