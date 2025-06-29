#include "vision.h"
#include "IMU_Task.h"
#include <stdio.h>


uint8_t  myUsbRxData[64] = { 0 };   // 接收到的数据
uint16_t myUsbRxNum = 0;            // 接收到的字节数

vision_rx Vision_Read_Data;
vision_tx Vision_Send_Data	;


void vision_receive_test(void){
	if(myUsbRxNum	!= 0){
		static char mystr[100]={0};
		sprintf(mystr,"\r\n收到%d个字节：\r内容是:%s\r\n",myUsbRxNum,(char*)myUsbRxData);
		CDC_Transmit_FS((uint8_t*)mystr,strlen(mystr));
		myUsbRxNum=0;

	}

}

uint8_t vision_read(uint8_t* data,uint32_t len){
	if(len == 32){
		memcpy(&Vision_Read_Data, data, len);
		if(Vision_Read_Data.start	== 's' &&	Vision_Read_Data.end == 'e'){	
		return 1;		
		}else{
		return 0;
		}
	}else{
		return 0;
	}
}

void vision_send(void){
	Vision_Send_Data.start 			= 		's';
	Vision_Send_Data.end 				= 		'e';
	Vision_Send_Data.type  						= 		0xB0;
	Vision_Send_Data.yaw 							=			IMU_Chass_MSG.angle[0]/180.0*3.141593;
	Vision_Send_Data.pitch 						=			IMU_Chass_MSG.angle[1]/180.0*3.141593;
	Vision_Send_Data.enemy_team_color	= 		1;
	Vision_Send_Data.mode							=			0;
	Vision_Send_Data.rune_flag				=			0;

	static uint8_t mystr[60]={0};
	memcpy(&mystr, &Vision_Send_Data, 32);
	CDC_Transmit_FS(mystr,32);
}

void vision_clear_R(void){
	memset(&Vision_Read_Data,0,32);
}
void vision_clear_S(void){
memset(&Vision_Send_Data,0,32);
}