//
// Created by zhangtianyi on 2022/3/23.
//

#include "uart.h"

#include<cstring>

//void NUC_date_Decode(uint8_t *buff)
//{
//    frame_info.head.sof = 0x5A;
//    frame_info.head.data_len =;
//    frame_info.head.seq =;
//    frame_info.head.crc8 = ;
//    frame_info.cmd_id=;
//    frame_info.frame_tail[2]
//}

void Robot_Status (uint8_t* data)
{
  //初始化字节索引
  unsigned int index = 0;
  //写入数据帧的帧头
  *(data + index) = 0xA5;
}
//void Referee_Date_Send(uint16_t cmd_id,uint8_t* p_data)
void Referee_Transmit_BetweenCar(uint16_t cmd_id,uint8_t* p_data)
{
    static uint8_t seq=0;
    uint8_t Tx_Buff[200];
    memset(Tx_Buff,0,200);

    //帧头打包
    Tx_Buff[0] = 0xA5;
    memcpy(&Tx_Buff[1],(uint8_t*)sizeof(p_data),2);
    Tx_Buff[3]=seq;
    Get_CRC8_Check_Sum(Tx_Buff,5,2);

    //命令码加入
    memcpy(&Tx_Buff[5],(uint8_t*)&cmd_id,2);
    memcpy(&Tx_Buff[7],p_data,(uint8_t)sizeof(p_data));
    Get_CRC16_Check_Sum(Tx_Buff,(uint8_t)sizeof(p_data) +9,3);
    if(seq == 0xff){seq=0;}
    else seq++;

    //数据上传
//    USAPT_ClearFlag(USART_Referee,USART_FLAG_TG);
//    for(int i=0;i<(uint8_t)sizeof(p_data)+9;i++)
//    {
//        USART_SendDate(USART_Referee,Tx_Buff[i]);
//        while(USART_GetFlagStatus(USART_Referee,USART_fLAG_Tc) == RESET);
//    }data
}


int main(int argc, char **argv)
{

}