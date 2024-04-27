//
// Created by zhangtianyi on 2022/3/21.
//

#include "uart.h"
#include "string.h"
#include <stdint.h>

//uint8_t User_meta_data[256]__attribute__((at(0x24038000)));//裁判系统接收存储数组
//uint8_t referee_meta_data[256]__attribute__((at(0x24028000)));
//uint8_t * Vehicle_Communicate_data __attribute__((at(0x24042000)));



//frame frame_info;



uint8_t ref_packge[packs][max_single_pack_len];  //最多一次收10个包


ext_game_status_t                      game_state;//比赛状态数据， 1Hz 周期发送
ext_game_result_t                      game_result;//比赛结果数据，比赛结束后发送
ext_game_robot_HP_t             			 game_robot_hp;//机器人血量，1Hz 周期发送

ext_dart_status_t    			  				 	 dart_state;//飞镖发射状态，飞镖发射后发送 范围所有机器人
ext_event_data_t    								   event_data;//场地事件数据，事件改变后发送
ext_supply_projectile_action_t         supply_projectile_action;//补给站动作标识 动作改变后发送 己方机器人
ext_referee_warning_t									 referee_warning;//裁判警告信息 警告发生后发送
ext_dart_remaining_time_t       			 dart_remaining_time;//飞镖发射口倒计时 1Hz 周期发送 己方机器人

ext_ICRA_buff_debuff_zone_status_t     ICRA_buff_debuff_zone;//人工智能挑战赛加成与惩罚区状态 1Hz 周期发送 己方机器人

ext_game_robot_status_t                game_robot_state;//机器人状态数据， 10Hz 周期发送
ext_power_heat_data_t                  power_heat_data;//实时功率热量数据， 50Hz 周期发送
ext_game_robot_pos_t                   game_robot_pos;//机器人位置数据， 10Hz 发送
ext_buff_t                             buff_musk;//机器人增益数据,增益状态改变后发送

aerial_robot_energy_t                  aerial_robot_energy;//空中机器人能量状态 10Hz
ext_robot_hurt_t                       robot_hurt;//伤害状态数据，伤害发生后发送
ext_shoot_data_t										   shoot_data;//实时射击数据，子弹发射后发送
ext_bullet_remaining_t								 bullet_remaining;//子弹剩余发射数 10Hz 所有机器人发送
ext_rfid_status_t 										 rfid_status;//机器人 RFID 状态 1Hz 单一机器人
ext_dart_client_cmd_t                  dart_client_cmd;//飞镖机器人客户端指令数据 10Hz 单一机器人


update_event 													 renew;//裁判系统更新消息
user_buff_judege											 buff;
user_robot_hurt_judege								 hurt;
user_robot_RFID_judege 								 rfid;




ext_student_interactive_header_data_t student_interactive_header_data;//机器人间交互数据接收信息






/******************************************************************
函数名；Referee_Data_Diapcak
功能：  裁判系统数据验证＋解算
参数；  uint8_t *data,uint8_t this_time_len
返回值：无
下级函数：_Data_Diapcak
上级函数：USART3_IDLE_CALLBACK
******************************************************************/


void Referee_Data_Diapcak(uint8_t *data,uint8_t this_time_len)
{
    uint32_t Verify_CRC8_OK;
    uint32_t Verify_CRC16_OK;
    uint8_t i=0;
    uint8_t pack_size=0;
    do{
        if( *data==0xA5)
        {
            frame_info.head.sof=*data;
            frame_info.head.data_len=*(data+2)<<8| *(data+1);
            frame_info.head.seq=*(data+3);
            frame_info.head.crc8=*(data+4);
            frame_info.cmd_id=*(data+6)<<8|*(data+5);
            Verify_CRC8_OK=Verify_CRC8_Check_Sum(data, frame_header_len);
            Verify_CRC16_OK= Verify_CRC16_Check_Sum(data,pack_len);

            if((Verify_CRC8_OK==1)&&(Verify_CRC16_OK==1))  //校验通过
            {
                memcpy(ref_packge[i],data,7+frame_info.head.data_len+2);
                _Data_Diapcak(ref_packge[i]);
                i++;
            }

        }
        data+=(7+frame_info.head.data_len+2);
        pack_size+=(7+frame_info.head.data_len+2);

    }while(pack_size<this_time_len);


}



/******************************************************************
函数名；_Data_Diapcak
功能：  裁判系统数据解算+映射到对应结构体成员
参数；  uint8_t *pdata
返回值：无
上级函数：Referee_Data_Diapcak
******************************************************************/
void _Data_Diapcak(uint8_t *pdata)
{
    float temp_power,synctime;
    uint16_t cmd_id;

    cmd_id=*(pdata+6)<<8|*(pdata+5);

    switch(cmd_id)
    {


        case 0x001:
            game_state.game_type = (*(pdata+data_addr)&0x0f);
            game_state.game_progress = (*(pdata+data_addr)>>4);
            game_state.stage_remain_time = *(pdata+data_addr+2)<<8|*(pdata+data_addr+1);//当前阶段剩余时间，单位 s

            BYTE0(game_state.SyncTimeStamp) = *(pdata+data_addr+3);
            BYTE1(game_state.SyncTimeStamp) = *(pdata+data_addr+4);
            BYTE2(game_state.SyncTimeStamp) = *(pdata+data_addr+5);
            BYTE3(game_state.SyncTimeStamp) = *(pdata+data_addr+6);
            BYTE4(game_state.SyncTimeStamp) = *(pdata+data_addr+7);
            BYTE5(game_state.SyncTimeStamp) = *(pdata+data_addr+8);
            BYTE6(game_state.SyncTimeStamp) = *(pdata+data_addr+9);
            BYTE7(game_state.SyncTimeStamp) = *(pdata+data_addr+10);

            break;



        case 0x002:
            game_result.winner = *(pdata+data_addr);
            break ;



        case 0x003:
            game_robot_hp.red_1_robot_HP = *(pdata+data_addr+1)<<8|*(pdata+data_addr);
            game_robot_hp.red_2_robot_HP = *(pdata+data_addr+3)<<8|*(pdata+data_addr+2);
            game_robot_hp.red_3_robot_HP = *(pdata+data_addr+5)<<8|*(pdata+data_addr+4);
            game_robot_hp.red_4_robot_HP = *(pdata+data_addr+7)<<8|*(pdata+data_addr+6);
            game_robot_hp.red_5_robot_HP = *(pdata+data_addr+9)<<8|*(pdata+data_addr+8);
            game_robot_hp.red_7_robot_HP = *(pdata+data_addr+11)<<8|*(pdata+data_addr+10);
            game_robot_hp.blue_outpost_HP = *(pdata+data_addr+13)<<8|*(pdata+data_addr+12);
            game_robot_hp.red_base_HP = *(pdata+data_addr+15)<<8|*(pdata+data_addr+14);
            game_robot_hp.blue_1_robot_HP = *(pdata+data_addr+17)<<8|*(pdata+data_addr+16);
            game_robot_hp.blue_2_robot_HP = *(pdata+data_addr+19)<<8|*(pdata+data_addr+18);
            game_robot_hp.blue_3_robot_HP = *(pdata+data_addr+21)<<8|*(pdata+data_addr+20);
            game_robot_hp.blue_4_robot_HP = *(pdata+data_addr+23)<<8|*(pdata+data_addr+22);
            game_robot_hp.blue_5_robot_HP = *(pdata+data_addr+25)<<8|*(pdata+data_addr+24);
            game_robot_hp.blue_7_robot_HP = *(pdata+data_addr+27)<<8|*(pdata+data_addr+26);
            game_robot_hp.blue_outpost_HP = *(pdata+data_addr+29)<<8|*(pdata+data_addr+28);
            game_robot_hp.blue_base_HP = *(pdata+data_addr+31)<<8|*(pdata+data_addr+30);


        case 0x004:
            dart_state.dart_belong = *(pdata+data_addr);
            dart_state.stage_remaining_time = *(pdata+data_addr+2)<<8|*(pdata+data_addr+1);
            break;



        case 0x005:
            // F1-F6 激活状态
            ICRA_buff_debuff_zone.F1_zone_status = *(pdata+data_addr)&0x01;

            ICRA_buff_debuff_zone.F2_zone_status = ((*(pdata+data_addr)&0x10)>>4);

            ICRA_buff_debuff_zone.F3_zone_status = (*(pdata+data_addr+1)&0x01);

            ICRA_buff_debuff_zone.F4_zone_status = ((*(pdata+data_addr+1)&0x10)>>4);

            ICRA_buff_debuff_zone.F2_zone_status = ((*(pdata+data_addr+2)&0x01));

            ICRA_buff_debuff_zone.F2_zone_status = ((*(pdata+data_addr+2)&0x10)>>4);

            ICRA_buff_debuff_zone.red1_bullet_left = *(pdata+data_addr+4)<<8|*(pdata+data_addr+3);
            ICRA_buff_debuff_zone.red2_bullet_left = *(pdata+data_addr+6)<<8|*(pdata+data_addr+5);
            ICRA_buff_debuff_zone.blue1_bullet_left = *(pdata+data_addr+8)<<8|*(pdata+data_addr+7);
            ICRA_buff_debuff_zone.blue2_bullet_left = *(pdata+data_addr+10)<<8|*(pdata+data_addr+9);
            break;



        case 0x101:
            event_data.status_replenishmentONE = (*(pdata+data_addr)&0x01);
            event_data.status_replenishmentTWO = ((*(pdata+data_addr)>>1)&0x01);
            event_data.status_replenishmentTHREE = ((*(pdata+data_addr)>>2)&0x01);
            event_data.Hit_point = ((*(pdata+data_addr)>>3)&0x01);
            event_data.Low_power_mechanism = ((*(pdata+data_addr)>>4)&0x01);
            event_data.High_power_mechanism = ((*(pdata+data_addr)>>5)&0x01);
            event_data.R2_ring = ((*(pdata+data_addr)>>6)&0x01);
            event_data.R3_ring = ((*(pdata+data_addr)>>7)&0x01);
            event_data.R4_ring = (*(pdata+data_addr+1)&0x01);
            event_data.Base_shield = ((*(pdata+data_addr+1)>>1)&0x1);
            event_data.skirmish = ((*(pdata+data_addr+1)>>2)&0x01);
            break;



        case 0x102:
            supply_projectile_action.supply_projectile_id = *(pdata+data_addr);
            supply_projectile_action.supply_robot_id = *(pdata+data_addr+1);
            supply_projectile_action.supply_projectile_step = *(pdata+data_addr+2);
            supply_projectile_action.supply_projectile_num = *(pdata+data_addr+3);
            break;



        case 0x104:
            referee_warning.level = *(pdata+data_addr);
            referee_warning.foul_robot_id = *(pdata+data_addr+1);
            break;



        case 0x105:
            dart_remaining_time.dart_remaining_time = *(pdata+data_addr);
            break;



        case 0x201:
            game_robot_state.robot_id=*(pdata+data_addr);
            game_robot_state.robot_level=*(pdata+data_addr+1);
            game_robot_state.remain_HP=*(pdata+data_addr+3)<<8|*(pdata+data_addr+2);
            game_robot_state.max_HP=*(pdata+data_addr+5)<<8|*(pdata+data_addr+4);
            game_robot_state.shooter_id1_17mm_cooling_rate=*(pdata+data_addr+7)<<8|*(pdata+data_addr+6);
            game_robot_state.shooter_id1_17mm_cooling_limit=*(pdata+data_addr+9)<<8|*(pdata+data_addr+8);
            game_robot_state.shooter_id1_17mm_speed_limit=*(pdata+data_addr+11)<<8|*(pdata+data_addr+10);
            game_robot_state.shooter_id2_17mm_cooling_rate=*(pdata+data_addr+13)<<8|*(pdata+data_addr+12);
            game_robot_state.shooter_id2_17mm_cooling_limit=*(pdata+data_addr+15)<<8|*(pdata+data_addr+14);
            game_robot_state.shooter_id2_17mm_speed_limit=*(pdata+data_addr+17)<<8|*(pdata+data_addr+16);
            game_robot_state.shooter_id1_42mm_cooling_rate=*(pdata+data_addr+19)<<8|*(pdata+data_addr+18);
            game_robot_state.shooter_id1_42mm_cooling_limit=*(pdata+data_addr+21)<<8|*(pdata+data_addr+20);
            game_robot_state.shooter_id1_42mm_speed_limit=*(pdata+data_addr+23)<<8|*(pdata+data_addr+22);
            game_robot_state.chassis_power_limit=*(pdata+data_addr+25)<<8|*(pdata+data_addr+24);
            game_robot_state.mains_power_gimbal_output = (*(pdata+data_addr+26)&0x01);
            game_robot_state.mains_power_chassis_output = ((*(pdata+data_addr+26)&0x02)>>1);
            game_robot_state.mains_power_shooter_output = ((*(pdata+data_addr+26)&0x04)>>2);
            break;



//        case 0x202:
//            Hero_Chassis.Flag.power_heat_update=1;
//            power_heat_data.chassis_volt=*(pdata+data_addr+1)<<8|*(pdata+data_addr);    //ma
//            power_heat_data.chassis_current=*(pdata+data_addr+3)<<8|*(pdata+data_addr+2);//mv
//
//            BYTE0(power_heat_data.chassis_power) = *(pdata+data_addr+4);
//            BYTE1(power_heat_data.chassis_power) = *(pdata+data_addr+5);
//            BYTE2(power_heat_data.chassis_power) = *(pdata+data_addr+6);
//            BYTE3(power_heat_data.chassis_power) = *(pdata+data_addr+7);
//				power_heat_data.chassis_power=temp_power;

            power_heat_data.chassis_power_buffer=*(pdata+data_addr+9)<<8|*(pdata+data_addr+8);
            power_heat_data.shooter_id1_17mm_cooling_heat=*(pdata+data_addr+11)<<8|*(pdata+data_addr+10);
            power_heat_data.shooter_id2_17mm_cooling_heat=*(pdata+data_addr+13)<<8|*(pdata+data_addr+12);
            power_heat_data.shooter_id1_42mm_cooling_heat=*(pdata+data_addr+15)<<8|*(pdata+data_addr+14);

            renew.power_heat_data.chassis_power=1;
            renew.power_heat_data.chassis_power_buffer=1;
            renew.power_heat_data.shooter_heat0=1;
            //Muzzle.cooling_times=1;
//			if(game_robot_state.remain_HP<game_robot_state.max_HP*0.2)//血量低于20% buff.heat_cooling_buff==1
//			{
//				Muzzle.cooling_times=2;
//			}
//			if(buff.defense_buff==1&&buff.heat_cooling_buff==1)// 有冷却 有防御
//			{
//				Muzzle.cooling_times=3;
//			}
//			if(buff.heat_cooling_buff==1&&buff.defense_buff==0&&game_robot_state.remain_HP>game_robot_state.max_HP*0.2)//有冷却 无防御 血量>20%
//			{
//				Muzzle.cooling_times=5;
//			}
//			if(buff.heat_cooling_buff==0)
//			{
//				Muzzle.cooling_times=1;
//			}

/**************发送电容控制信号 频率跟裁判系统功率更新频率一致*********************/
//				if(Supper_Cap.voltage<CAP_LOW_VOLTAGE) Supper_Cap.no_power_flag=1;  //电量检测
//				if(Supper_Cap.voltage>CAP_HIGH_VOLTAGE)Supper_Cap.no_power_flag=0;
//					    if(RC_Ctl.key_shift_flag && Supper_Cap.offline_flag==0 && Supper_Cap.no_power_flag==0) //需要加速 && 可以加速
//							{
//
//								Cap_Off_Cnt=0;
//
//
////								if(power_heat_data.chassis_power_buffer<55) Supper_Cap.input_mode=1; //吃焦耳不充
////								else Supper_Cap.input_mode=0;
//								Supper_Cap.input_mode=0;//固定充电
//								Supper_Cap.power_mode=1; // 固定放电
//
//								P_part=0;  //清除功率环数据
//								I_part=0;
//								last_error=0;
//
//								Target_Sum_Limit=9500;  //还原电流和速度
//								Out_Current_Limit=10000;
//
//							}
//
//							else //不加速状态
//							{
////								Cap_Off_Cnt++;//延时计时
////
////								if(Cap_Off_Cnt<20)
////								Supper_Cap.power_mode=1;//放电补偿
////								else Supper_Cap.power_mode=0;

//								Supper_Cap.power_mode=0;//固定关闭电容
//
//								Target_Sum_Limit=7000; //还原目标速度
//
//								Out_Current_Limit=Power_Control_1(power_heat_data.chassis_power,power_heat_data.chassis_power_buffer);//功率控制
//
////								/////////////////////////////////////////////////////////////////        充电条件（很重要） 1.爬坡=12°   2.加速不充   3.刹车不充
////								if(( abs(Motor_Info[0].target)+abs(Motor_Info[1].target)+abs(Motor_Info[2].target)+abs(Motor_Info[3].target))<2000 ) //刹车
////								{
////									if(abs(( abs(Motor_Info[0].speed_raw)+abs(Motor_Info[1].speed_raw)+abs(Motor_Info[2].speed_raw)+abs(Motor_Info[3].speed_raw)) - ( abs(Motor_Info[0].target)+abs(Motor_Info[1].target)+abs(Motor_Info[2].target)+abs(Motor_Info[3].target)))<3000)
////									{
////										Supper_Cap.input_mode=0;//充  //已停
////									}
////									else
////									{
////										Supper_Cap.input_mode=1;//不充
////									}
////								}
////								else if(( abs(Motor_Info[0].target)+abs(Motor_Info[1].target)+abs(Motor_Info[2].target)+abs(Motor_Info[3].target))<18000  )Supper_Cap.input_mode=0;//其他情况
////
////								else Supper_Cap.input_mode=1;//加速
////
////
////								if(Holder.Pitch_6050_Angle>12.0f) Supper_Cap.input_mode=1;//爬坡强制关闭充电（优先级更高 ）
//								if(power_heat_data.chassis_power_buffer<55) Supper_Cap.input_mode=1; //吃焦耳不充
//								else Supper_Cap.input_mode=0;
//							}

            //	FDCAN2_send_msg_Supercap(1); //发送信息给  Supercap
            break;



        case 0x203:
            BYTE0(game_robot_pos.x) = *(pdata+data_addr);
            BYTE1(game_robot_pos.x) = *(pdata+data_addr+1);
            BYTE2(game_robot_pos.x) = *(pdata+data_addr+2);
            BYTE3(game_robot_pos.x) = *(pdata+data_addr+3);

            BYTE0(game_robot_pos.y) = *(pdata+data_addr+4);
            BYTE1(game_robot_pos.y) = *(pdata+data_addr+5);
            BYTE2(game_robot_pos.y) = *(pdata+data_addr+6);
            BYTE3(game_robot_pos.y) = *(pdata+data_addr+7);

            BYTE0(game_robot_pos.z) = *(pdata+data_addr+8);
            BYTE1(game_robot_pos.z) = *(pdata+data_addr+9);
            BYTE2(game_robot_pos.z) = *(pdata+data_addr+10);
            BYTE3(game_robot_pos.z) = *(pdata+data_addr+11);

            BYTE0(game_robot_pos.yaw) = *(pdata+data_addr+12);
            BYTE1(game_robot_pos.yaw) = *(pdata+data_addr+13);
            BYTE2(game_robot_pos.yaw) = *(pdata+data_addr+14);
            BYTE3(game_robot_pos.yaw) = *(pdata+data_addr+15);
            break;



        case 0x204:
            buff_musk.power_rune_buff=*(pdata+data_addr);

            if((buff_musk.power_rune_buff & 0x01)==0x01)	buff.recover_buff=1;
            else if((buff_musk.power_rune_buff & 0x02)==0x02) buff.heat_cooling_buff=1;
            else if((buff_musk.power_rune_buff & 0x04)==0x04) buff.defense_buff=1;
            else if((buff_musk.power_rune_buff & 0x08)==0x08) buff.attack_buff=1;
            else
            {
                buff.attack_buff=0;
                buff.defense_buff=0;
                buff.heat_cooling_buff=0;
                buff.recover_buff=0;
            }

            /*********************以下内容与热量控制部分交互**************************/
            ////		/*枪口热量判断任然存在疑问*/
            if(buff.heat_cooling_buff==1)
            {
                //	Muzzle.cooling_times=2;

                if(buff.defense_buff==1)
                {
                    //	Muzzle.cooling_times=3;
                }
                else if(buff.defense_buff==0)
                {
                    if(game_robot_state.remain_HP>game_robot_state.max_HP*0.2)
                    {
                        //	Muzzle.cooling_times=5;
                    }
                    else if(game_robot_state.remain_HP<game_robot_state.max_HP*0.2)
                    {
                        //					if(rfid.energy_agency_buff||rfid.base_buff||rfid.upland_buff)//高地部分的增益判断有问题
                        //					{
                        //							Muzzle.cooling_times=5;
                        //					}
                    }
                }
            }
            else
            {
                //Muzzle.cooling_times=1;
            }
/*********************以上内容与热量控制部分交互 根据buff情况判断**************************/

            break;



        case 0x205:
            aerial_robot_energy.attack_time = *(pdata+data_addr);
            break;

        case 0x206:  //缺少更新标志位
            robot_hurt.armor_id = *(pdata+data_addr)&0xf;
            robot_hurt.hurt_type = *(pdata+data_addr)>>4;
            hurt.hurt_cnt = 0; //
            if(robot_hurt.hurt_type == 0x00)
            {
                hurt.hurt_armor = 1;
                hurt.hurt_flag = 1;
            }
            else hurt.hurt_armor = 0;


            if((robot_hurt.armor_id & 0x01)==0x01)
            {
                hurt.hurt_armor_id|=0x01;
            }
            else if((robot_hurt.armor_id & 0x02)==0x02)
            {
                hurt.hurt_armor_id|=0x02;
            }
            else if((robot_hurt.armor_id & 0x03)==0x03)
            {
                hurt.hurt_armor_id|=0x03;
            }
            break;

        case 0x207:
//		/*射速相关数据*/
            shoot_data.shooter_id =	*(pdata+data_addr+1);
            shoot_data.bullet_freq = *(pdata+data_addr+2);
            BYTE0(shoot_data.bullet_speed) = *(pdata+data_addr+3);
            BYTE1(shoot_data.bullet_speed) = *(pdata+data_addr+4);
            BYTE2(shoot_data.bullet_speed) = *(pdata+data_addr+5);
            BYTE3(shoot_data.bullet_speed) = *(pdata+data_addr+6);

            //	shoot.Shoot_Bullet_Sum++;

#ifdef Heat_Test
            //		if(shoot.Shoot_Bullet_Sum!=shoot.Last_Shoot_Bullet_Sum)
				 Muzzle.on_time_heat+=10;

		//		shoot.Last_Shoot_Bullet_Sum=shoot.Shoot_Bullet_Sum;
#else

#endif
            break;

        case 0x208:
            bullet_remaining.bullet_remaining_num_17mm = *(pdata+data_addr+1)<<8|*(pdata+data_addr);
            bullet_remaining.bullet_remaining_num_42mm = *(pdata+data_addr+3)<<8|*(pdata+data_addr+2);
            break;

        case 0x209:
            BYTE0(rfid_status.rfid_status) = *(pdata+data_addr);
            BYTE1(rfid_status.rfid_status) = *(pdata+data_addr+1);
            BYTE2(rfid_status.rfid_status) = *(pdata+data_addr+2);
            BYTE3(rfid_status.rfid_status) = *(pdata+data_addr+3);
            if((rfid_status.rfid_status&0x0001)==0x0001) rfid.base_buff=1;
            else if((rfid_status.rfid_status&0x0002)==0x0002) rfid.upland_buff=1;
            else if((rfid_status.rfid_status&0x0004)==0x0004) rfid.energy_agency_buff=1;
            else if((rfid_status.rfid_status&0x0008)==0x0008) rfid.fly_slope_buff=1;
            else if((rfid_status.rfid_status&0x0010)==0x0010)  rfid.outpost_buff=1;
            else if((rfid_status.rfid_status&0x0020)==0x0020) rfid.resource_island_buff=1;
            else if((rfid_status.rfid_status&0x0040)==0x0040) rfid.recover_blood_buff=1;
            else if((rfid_status.rfid_status&0x0080)==0x0080) rfid.engineer_blood_card_buff=1;
            else
            {
                rfid.base_buff=0;
                rfid.upland_buff=0;
                rfid.energy_agency_buff=0;
                rfid.fly_slope_buff=0;
                rfid.outpost_buff=0;
                rfid.resource_island_buff=0;
                rfid.recover_blood_buff=0;
                rfid.engineer_blood_card_buff=0;
            }
            break;

            //飞镖机器人客户端指令数据 10Hz 单一机器人
        case 0x20A:
            dart_client_cmd.dart_launch_opening_status = *(pdata+data_addr);
            dart_client_cmd.dart_attack_target = *(pdata+data_addr+1);
            dart_client_cmd.target_change_time = *(pdata+data_addr+3)<<8|*(pdata+data_addr+2);
            dart_client_cmd.first_dart_speed = *(pdata+data_addr+4);
            dart_client_cmd.second_dart_speed = *(pdata+data_addr+5);
            dart_client_cmd.third_dart_speed = *(pdata+data_addr+6);
            dart_client_cmd.fourth_dart_speed = *(pdata+data_addr+7);
            dart_client_cmd.last_dart_launch_time = *(pdata+data_addr+9)<<8|*(pdata+data_addr+8);
            dart_client_cmd.last_dart_launch_time = *(pdata+data_addr+11)<<8|*(pdata+data_addr+10);
            break;
    }







//	   if(cmd_id==0x101)//能量机关
//		 {
//			//event_data.event_type
//		 }
//		if(cmd_id==0x201)
//	 	{
//			game_robot_state.robot_id=*(pdata+data_addr);
//			game_robot_state.robot_level=*(pdata+data_addr+1);
//
//			game_robot_state.remain_HP=*(pdata+data_addr+3)<<8|*(pdata+data_addr+2);
//			game_robot_state.max_HP=*(pdata+data_addr+5)<<8|*(pdata+data_addr+4);
//			game_robot_state.shooter_id1_17mm_cooling_rate=*(pdata+data_addr+7)<<8|*(pdata+data_addr+6);
//			game_robot_state.shooter_id1_17mm_cooling_limit=*(pdata+data_addr+9)<<8|*(pdata+data_addr+8);
//			game_robot_state.shooter_id1_17mm_speed_limit=*(pdata+data_addr+11)<<8|*(pdata+data_addr+10);
//			game_robot_state.shooter_id2_17mm_cooling_rate=*(pdata+data_addr+13)<<8|*(pdata+data_addr+12);
//			game_robot_state.shooter_id2_17mm_cooling_limit=*(pdata+data_addr+15)<<8|*(pdata+data_addr+14);
//			game_robot_state.shooter_id2_17mm_speed_limit=*(pdata+data_addr+17)<<8|*(pdata+data_addr+16);
////			game_robot_state.shooter_id1_42mm_cooling_rate=*(pdata+data_addr+19)<<8|*(pdata+data_addr+18);
////			game_robot_state.shooter_id1_42mm_cooling_limit=*(pdata+data_addr+21)<<8|*(pdata+data_addr+20);
////			game_robot_state.shooter_id1_42mm_speed_limit=*(pdata+data_addr+23)<<8|*(pdata+data_addr+22);
//			game_robot_state.chassis_power_limit=*(pdata+data_addr+19)<<8|*(pdata+data_addr+18);
////			game_robot_state.mains_power_chassis_output=*(pdata+data_addr+20)&0x01;
//			//game_robot_state.robot_level=2;
//
//			/***************************来不及修改 暂时注释掉************************************/
////			if(game_robot_state.max_chassis_power==(uint8_t)50)
////			{
////					RC_Speed.RC_max_speed=8000;
////			}
////			else if(game_robot_state.max_chassis_power==(uint8_t)60)
////			{
////					RC_Speed.RC_max_speed=8000;
////			}
////			else if(game_robot_state.max_chassis_power==(uint8_t)70)
////			{
////					RC_Speed.RC_max_speed=8000;
////			}
////			else if(game_robot_state.max_chassis_power==(uint8_t)100)
////			{
////					RC_Speed.RC_max_speed=8000;
////			}
////			else
////			{
////				  RC_Speed.RC_max_speed=8000;
////			}
//			/***************************来不及修改 暂时注释掉************************************/
//
//		 }
//		else if(cmd_id==0x202)
//		{
//			power_heat_data.chassis_volt=*(pdata+data_addr+1)<<8|*(pdata+data_addr);    //ma
//			power_heat_data.chassis_current=*(pdata+data_addr+3)<<8|*(pdata+data_addr+2);//mv
//
//				BYTE0(temp_power) = *(pdata+data_addr+4);
//				BYTE1(temp_power) = *(pdata+data_addr+5);
//				BYTE2(temp_power) = *(pdata+data_addr+6);
//				BYTE3(temp_power) = *(pdata+data_addr+7);
//
//				power_heat_data.chassis_power=temp_power;
//				power_heat_data.chassis_power_buffer=*(pdata+data_addr+9)<<8|*(pdata+data_addr+8);
//				power_heat_data.shooter_id1_17mm_cooling_heat=*(pdata+data_addr+11)<<8|*(pdata+data_addr+10);
//				power_heat_data.shooter_id1_42mm_cooling_heat=*(pdata+data_addr+15)<<8|*(pdata+data_addr+14);
//				power_heat_data.shooter_id2_17mm_cooling_heat=*(pdata+data_addr+15)<<8|*(pdata+data_addr+14);
//
//			/*功率 缓冲 17mm枪口热量更新标志*/
//			renew.power_heat_data.chassis_power=1;
//			renew.power_heat_data.chassis_power_buffer=1;
//			renew.power_heat_data.shooter_heat0=1;
//		}
//		else if(cmd_id==0x203)
//		{
//				BYTE0(game_robot_pos.x) = *(pdata+data_addr);
//				BYTE1(game_robot_pos.x) = *(pdata+data_addr+1);
//				BYTE2(game_robot_pos.x) = *(pdata+data_addr+2);
//				BYTE3(game_robot_pos.x) = *(pdata+data_addr+3);
//
//				BYTE0(game_robot_pos.y) = *(pdata+data_addr+4);
//				BYTE1(game_robot_pos.y) = *(pdata+data_addr+5);
//				BYTE2(game_robot_pos.y) = *(pdata+data_addr+6);
//				BYTE3(game_robot_pos.y) = *(pdata+data_addr+7);
//
//				BYTE0(game_robot_pos.z) = *(pdata+data_addr+8);
//				BYTE1(game_robot_pos.z) = *(pdata+data_addr+9);
//				BYTE2(game_robot_pos.z) = *(pdata+data_addr+10);
//				BYTE3(game_robot_pos.z) = *(pdata+data_addr+11);
//
//				BYTE0(game_robot_pos.yaw) = *(pdata+data_addr+12);
//				BYTE1(game_robot_pos.yaw) = *(pdata+data_addr+13);
//				BYTE2(game_robot_pos.yaw) = *(pdata+data_addr+14);
//				BYTE3(game_robot_pos.yaw) = *(pdata+data_addr+15);
//		}
//		else if(cmd_id==0x0204)
//		{
//			buff_musk.power_rune_buff=*(pdata+data_addr);
//
//			if((buff_musk.power_rune_buff & 0x01)==0x01)	buff.recover_buff=1;
//			else if((buff_musk.power_rune_buff & 0x02)==0x02) buff.heat_cooling_buff=1;
//			else if((buff_musk.power_rune_buff & 0x04)==0x04) buff.defense_buff=1;
//			else if((buff_musk.power_rune_buff & 0x08)==0x08) buff.attack_buff=1;
//			else
//			{
//				buff.attack_buff=0;
//				buff.defense_buff=0;
//				buff.heat_cooling_buff=0;
//				buff.recover_buff=0;
//			}
//		}
//		 else if(cmd_id==0x0205)
//		 {
//			aerial_robot_energy.attack_time = *(pdata+data_addr);
//		 }
//
//	 else	if(cmd_id==0x0206)
//		{
//
//			if((robot_hurt.armor_id & 0x01)==0x01)
//			{
//				hurt.hurt_armor_id|=0x01;
//			}
//      else if((robot_hurt.armor_id & 0x02)==0x02)
//			{
//				hurt.hurt_armor_id|=0x02;
//			}
//      else if((robot_hurt.armor_id & 0x03)==0x03)
//			{
//				hurt.hurt_armor_id|=0x03;
//			}

////
//////			robot_hurt.hurt_type=
//////      robot_hurt.armor_id
//		}
////
//	else if(cmd_id==0x0207)
//	{
//		/*射速相关数据*/
//		shoot_data.shooter_id =	*(pdata+data_addr+1);
//		shoot_data.bullet_freq = *(pdata+data_addr+2);
//		BYTE0(shoot_data.bullet_speed) = *(pdata+data_addr+3);
//		BYTE1(shoot_data.bullet_speed) = *(pdata+data_addr+4);
//		BYTE2(shoot_data.bullet_speed) = *(pdata+data_addr+5);
//		BYTE3(shoot_data.bullet_speed) = *(pdata+data_addr+6);

//		shoot.Shoot_Bullet_Sum++;
//
//		#ifdef Heat_Test
////		if(shoot.Shoot_Bullet_Sum!=shoot.Last_Shoot_Bullet_Sum)
//		 Muzzle.on_time_heat+=10;
//
////		shoot.Last_Shoot_Bullet_Sum=shoot.Shoot_Bullet_Sum;
//		#else
//
//		#endif
//	}
//	else if(cmd_id==0x0208)
//	{
//		bullet_remaining.bullet_remaining_num_17mm = *(pdata+data_addr+1)<<8|*(pdata+data_addr);
//		bullet_remaining.bullet_remaining_num_42mm = *(pdata+data_addr+3)<<8|*(pdata+data_addr+2);
//
//	}
////
////		/*目前只能这样 车子血量尽量保持20%以上  */
/////*			if(game_robot_state.remain_HP<game_robot_state.max_HP*0.2)//血量低于20% buff.heat_cooling_buff==1
////			{
////				Muzzle.cooling_times=2;
////			}
////			if(buff.defense_buff==1&&buff.heat_cooling_buff==1)// 有冷却 有防御
////			{
////				Muzzle.cooling_times=3;
////			}
////			if(buff.heat_cooling_buff==1&&buff.defense_buff==0&&game_robot_state.remain_HP>game_robot_state.max_HP*0.2)//有冷却 无防御 血量>20%
////			{
////				Muzzle.cooling_times=5;
////			}
////			if(buff.heat_cooling_buff==0)
////			{
////				Muzzle.cooling_times=1;
////			}
////*/
////		/*
////		基地增益机制 单方增益
////		六边形 50%防御增益 3倍枪口冷却
////		哨兵轨道掩体 5倍枪口冷却增益
////		高地增益机制 双方增益(一方占领另一方无法同时占领)
////		各高低占领状态完全独立，互不关联
////
////		飞坡增益机制 双方增益
////		50%防御增益 持续20秒
////		250J缓冲能量增益
////		3倍枪口热量冷却增益 20秒
////		前哨站增益 单方增益
////		5倍枪口热量冷却 (飞镖命中暂时失效 持续30秒)
////		能量机关增益机制
////		5倍枪口热量冷却
////		*/
//////		else if(cmd_id==0x0209)//RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效果
//////	  {
//////			if((rfid_status.rfid_status&0x0001)==0x0001) rfid.base_buff=1;
//////			else if((rfid_status.rfid_status&0x0001)==0x0001) rfid.upland_buff=1;
//////				else if((rfid_status.rfid_status&0x0001)==0x0001) rfid.energy_agency_buff=1;
//////					else if((rfid_status.rfid_status&0x0001)==0x0001) rfid.fly_slope_buff=1;
//////						else if((rfid_status.rfid_status&0x0001)==0x0001)  rfid.outpost_buff=1;
//////							else if((rfid_status.rfid_status&0x0001)==0x0001) rfid.resource_island_buff=1;
//////								else if((rfid_status.rfid_status&0x0001)==0x0001) rfid.recover_blood_buff=1;
//////			            else if((rfid_status.rfid_status&0x0001)==0x0001) rfid.engineer_blood_card_buff=1;
//////			else
//////			{
//////				rfid.base_buff=0;
//////				rfid.upland_buff=0;
//////				rfid.energy_agency_buff=0;
//////				rfid.fly_slope_buff=0;
//////				rfid.outpost_buff=0;
//////				rfid.resource_island_buff=0;
//////				rfid.recover_blood_buff=0;
//////				rfid.engineer_blood_card_buff=0;
//////			}
//////		}
//////		else if(cmd_id==0x0301)//机器人间交互
//////		{
//////		robot_communicate.data_id=*(Vehicle_Data.data+1)<<8|*(Vehicle_Data.data);
//////		robot_communicate.sender_id=*(Vehicle_Data.data+3)<<8|*(Vehicle_Data.data+2);
//////		robot_communicate.receiver_id=*(Vehicle_Data.data+5)<<8|*(Vehicle_Data.data+4);
//////
//////		 if(robot_communicate.sender_id==0x0109)	//雷达 车间通信
//////		 {
//////
//////		 }
//////
//////		}
////		/*枪口热量判断任然存在疑问*/
////		if(buff.heat_cooling_buff==1)
////		{
////			  Muzzle.cooling_times=2;
////
////			if(buff.defense_buff==1)
////			{
////				Muzzle.cooling_times=3;
////			}
////			else if(buff.defense_buff==0)
////			{
////				if(game_robot_state.remain_HP>game_robot_state.max_HP*0.2)
////				{
////				Muzzle.cooling_times=5;
////				}
////				else if(game_robot_state.remain_HP<game_robot_state.max_HP*0.2)
////				{
//////					if(rfid.energy_agency_buff||rfid.base_buff||rfid.upland_buff)//高地部分的增益判断有问题
//////					{
//////							Muzzle.cooling_times=5;
//////					}
////				}
////			}
////		}
////		else
////		{
////		  	Muzzle.cooling_times=1;
////		}
//		}
}






//void USART3_IDLE_CALLBACK()
//{
//	assert_param(uart != NULL);
//	static uint16_t temp=0,rx_len=0;

//	if(( __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!= RESET))//idle标志被置位
//	{
//
//
//		__HAL_UART_CLEAR_IDLEFLAG(&huart3);																																//< 清楚idle标志位，防止再次进入中断
//		HAL_UART_DMAStop(&huart3);


//		temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// 获取DMA中未传输的数据个数
//		rx_len = 128 - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
//		//Referee_Data_Diapcak(referee_meta_data,rx_len);
//		Referee_Data_Diapcak(referee_meta_data,rx_len);
//	 __HAL_DMA_SET_COUNTER(&hdma_usart3_rx,128);
////				__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF0_4|DMA_FLAG_HTIF0_4);
////		DMA1_Stream4->NDTR = (uint32_t)128;
//	  __HAL_DMA_ENABLE(&hdma_usart3_rx);
//		HAL_UART_Receive_DMA(&huart3,referee_meta_data,128);//重新打开DMA接收
//	}
//}

//void USART3_IDLE_CALLBACK(void)
//{
//    static uint16_t temp,rx_len;
//
//    //HAL_UART_DMAStop(&huart3);
//    if(( __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!= RESET))//idle标志被置位
//    {
//
//        __HAL_DMA_DISABLE(&hdma_usart3_rx);
//        __HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除标志位
//        //(void)USART3->
//        __HAL_UART_CLEAR_OREFLAG(&huart3);
////    (void)huart3.Instance->CR1;
////		temp=huart3.Instance->RDR;
////		temp=huart3.Instance->ISR;
//        temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// 获取DMA中未传输的数据个数
//        rx_len = 128 - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
//        //Referee_Data_Diapcak(referee_meta_data,rx_len);
//        Referee_Data_Diapcak(referee_meta_data,rx_len);
//        __HAL_DMA_SET_COUNTER(&hdma_usart3_rx,128);
////				__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF0_4|DMA_FLAG_HTIF0_4);
////		DMA1_Stream4->NDTR = (uint32_t)128;
//        __HAL_DMA_ENABLE(&hdma_usart3_rx);
//        HAL_UART_Receive_DMA(&huart3,referee_meta_data,128);//重新打开DMA接收
//    }
//}



const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
        {
                0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
                0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
                0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
                0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
                0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
                0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
                0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
                0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
                0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
                0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
                0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
                0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
                0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
                0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
                0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
                0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
        };



unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}



unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}



uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
        {
                0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
                0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
                0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
                0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
                0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
                0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
                0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
                0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
                0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
                0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
                0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
                0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
                0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
                0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
                0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
                0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
                0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
                0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
                0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
                0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
                0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
                0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
                0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
                0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
                0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
                0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
                0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
                0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
                0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
                0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
                0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
                0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
        };



uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
                                                      0x00ff];
    }
    return wCRC;
}



uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;//FALSE;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
                                                              pchMessage[dwLength - 1]);
}