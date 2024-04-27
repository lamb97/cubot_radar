//
// Created by zhangtianyi on 2022/3/21.
//

#ifndef SRC_UART_H
#define SRC_UART_H
#include <stdio.h>
#include <stdint.h>
#define __packed __attribute__((packed))


typedef struct
{
    struct
    {
        uint8_t sof;
        uint16_t data_len;
        uint8_t seq;
        uint8_t crc8;
    }head;                                     //frame_header 格式

    uint16_t cmd_id;
    uint8_t  frame_tail[2];

}	frame;                                   //通信协议格式

frame frame_info;


typedef  struct
{
   uint16_t target_robot_ID;
   float target_position_x;
   float target_position_y;
}__packed ext_client_map_command_t;    //小地图接受信息标识0x0305,最大接受频率：10hz

typedef  struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
}__packed ext_game_robot_HP_t;                      //机器人血量数据：0x0003。发送频率：1Hz


typedef  struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
}__packed ext_game_status_t;                       //场地事件数据：0x0101。发送频率：事件改变后发送


typedef  struct
{
    uint8_t winner;
}__packed ext_game_result_t;                       //比赛结果数据：0x0002。发送频率：比赛结束后发送


typedef  struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
}__packed ext_dart_status_t;                        //飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人


typedef  struct
{
    uint8_t F1_zone_status:1;
    uint8_t F1_zone_buff_debuff_status:3;
    uint8_t F2_zone_status:1;
    uint8_t F2_zone_buff_debuff_status:3;
    uint8_t F3_zone_status:1;
    uint8_t F3_zone_buff_debuff_status:3;
    uint8_t F4_zone_status:1;
    uint8_t F4_zone_buff_debuff_status:3;
    uint8_t F5_zone_status:1;
    uint8_t F5_zone_buff_debuff_status:3;
    uint8_t F6_zone_status:1;
    uint8_t F6_zone_buff_debuff_status:3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
}__packed ext_ICRA_buff_debuff_zone_status_t;       //人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人


typedef  struct
{
    uint8_t status_replenishmentONE:1 ;
    uint8_t status_replenishmentTWO:1 ;
    uint8_t status_replenishmentTHREE:1 ;
    uint8_t Hit_point:1 ;
    uint8_t Low_power_mechanism:1 ;
    uint8_t High_power_mechanism:1 ;
    uint8_t R2_ring:1 ;
    uint8_t R3_ring:1 ;
    uint8_t R4_ring:1 ;
    uint8_t Base_shield:1 ;
    uint8_t skirmish:1 ;
}__packed ext_event_data_t;                         //场地事件数据：0x0101。发送频率：事件改变后发送


typedef  struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
}__packed ext_supply_projectile_action_t;           //补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人


typedef  struct
{
    uint8_t level;
    uint8_t foul_robot_id;
}__packed ext_referee_warning_t;                    //裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送


typedef  struct
{
    uint8_t dart_remaining_time;
}__packed ext_dart_remaining_time_t;               //飞镖发射口倒计时：0x0105。发送频率：1Hz 周期发送，发送范围：己方机器人


typedef  struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
}__packed ext_game_robot_status_t;               //比赛机器人状态：0x0201。发送频率：10Hz


typedef  struct
{
    uint16_t chassis_volt;//底盘输出电压 单位 毫伏
    uint16_t chassis_current;// 底盘输出电流 单位 毫安
    float chassis_power;// 底盘输出功率 单位 W 瓦

    uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
    uint16_t shooter_id1_17mm_cooling_heat;//1 号 17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;//2 号 17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;//42mm 枪口热量
}__packed ext_power_heat_data_t;                 //实时功率热量数据：0x0202。发送频率：50Hz


typedef  struct
{
    float x;
    float y;
    float z;
    float yaw;
}__packed ext_game_robot_pos_t;                  //机器人位置：0x0203。发送频率：10Hz


typedef  struct
{
    uint8_t power_rune_buff;
}__packed ext_buff_t;                             //机器人增益：0x0204。发送频率：1Hz


typedef  struct
{
    uint8_t attack_time;
}__packed aerial_robot_energy_t;                 //空中机器人能量状态：0x0205。发送频率：10Hz


typedef  struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
}__packed ext_robot_hurt_t;                      //伤害状态：0x0206。发送频率：伤害发生后发送


typedef  struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
}__packed ext_shoot_data_t;                      //实时射击信息：0x0207。发送频率：射击后发送


typedef  struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
}__packed ext_bullet_remaining_t;               //子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人发送


typedef  struct
{
    uint32_t rfid_status;
}__packed ext_rfid_status_t;                    //机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人


typedef  struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
}__packed ext_dart_client_cmd_t;                 //飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人


typedef  struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
}__packed ext_student_interactive_header_data_t;  //交互数据接收信息：0x0301


//typedef __packed struct
//	{
//uint8_t data[];
//} robot_interactive_data_t;              //交互数据 机器人间通信：0x0301





typedef  struct
{
    uint8_t operate_tpye;
    uint8_t layer;
}__packed ext_client_custom_graphic_delete_t;    //客户端删除图形 机器人间通信：0x0301


typedef  struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
}__packed graphic_data_struct_t;                 //图形数据


typedef  struct
{
    graphic_data_struct_t grapic_data_struct;
}__packed ext_client_custom_graphic_single_t;    //客户端绘制一个图形 机器人间通信：0x0301


typedef  struct
{
    graphic_data_struct_t grapic_data_struct[2];
}__packed ext_client_custom_graphic_double_t;    //客户端绘制三个图形 机器人间通信：0x0301


typedef  struct
{
    graphic_data_struct_t grapic_data_struct[5];
}__packed ext_client_custom_graphic_five_t;      //客户端绘制五个图形 机器人间通信：0x0301


typedef  struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
}__packed ext_client_custom_character_t;        //客户端绘制字符 机器人间通信：0x0301


typedef  struct
{
    graphic_data_struct_t grapic_data_struct[7];
}__packed ext_client_custom_graphic_seven_t;    //客户端绘制七个图形 机器人间通信：0x0301

/*
0x0200-0x02ff		6+n		唧唧通信
0x100						6+2		删除图形
0x101						6+15	绘制一个
0x102						6+30	两个
0x103						6+75	五个
0x104						6+105	七个
0x110						6+45	绘制字符图形

Robot id:
red
1 			hero
2 			engineer
3/4/5		infantry
6				fly
7				guard
9				radar
blue
101
102
103/4/5
106
107
109

Client id:
red
0x101 0x102
0x103 0x104 0x105
0x106

blue
0x165	0x166
0x167 0x168 0x169
0x16A
*/
typedef  struct
{
    uint8_t data[30];
}__packed robot_interactive_data_t;            //交互数据接收信息：0x0302。发送频率：上限 30Hz


typedef  struct
{
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
}__packed ext_robot_command_t;                 //小地图交互信息标识：0x0303。发送频率：触发时发送。


typedef struct
{
    struct
    {
        uint8_t remain_HP;
        uint8_t max_HP;
        uint8_t shooter_heat0_cooling_rate;
        uint8_t shooter_heat0_cooling_limit;
    }game_robot_status;
    struct
    {
        uint8_t chassis_power;
        uint8_t chassis_power_buffer;
        uint8_t shooter_heat0;
    }power_heat_data;
    struct
    {
        uint8_t power_rune_buff;
    }buff;

}update_event;



/*用户机器人增益判断*/
typedef struct
{
    uint8_t recover_buff;
    uint8_t heat_cooling_buff;
    uint8_t defense_buff;
    uint8_t attack_buff;

}user_buff_judege;



typedef struct
{
    uint8_t hurt_armor_id;
    uint8_t hurt_armor;
    uint8_t module_offline;
    uint8_t overspeed;
    uint8_t overchassis_power;
    uint8_t hurt_strike;
    uint8_t hurt_flag;
    uint8_t hurt_flag_last;
    uint16_t hurt_cnt;
    uint16_t hurt_reaction_cnt;   //反应计数
}
      user_robot_hurt_judege;



typedef struct
{
    uint8_t base_buff;
    uint8_t upland_buff;
    uint8_t energy_agency_buff;
    uint8_t fly_slope_buff;
    uint8_t outpost_buff;
    uint8_t resource_island_buff;
    uint8_t recover_blood_buff;
    uint8_t engineer_blood_card_buff;
}
        user_robot_RFID_judege;



#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
#define BYTE4(dwTemp)       (*((char *)(&dwTemp) + 4))
#define BYTE5(dwTemp)       (*((char *)(&dwTemp) + 5))
#define BYTE6(dwTemp)       (*((char *)(&dwTemp) + 6))
#define BYTE7(dwTemp)       (*((char *)(&dwTemp) + 7))

#define data_addr 7
#define frame_header_len 5
#define pack_len 7+frame_info.head.data_len+2
#define max_single_pack_len 50
#define packs 10




uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);


void Referee_Data_Diapcak(uint8_t *data,uint8_t this_time_len);
void _Data_Diapcak(uint8_t *pdata);



extern ext_game_status_t                      game_state;//比赛状态数据， 1Hz 周期发送
extern ext_game_result_t                      game_result;//比赛结果数据，比赛结束后发送
extern ext_game_robot_HP_t             			 game_robot_hp;//机器人血量，1Hz 周期发送


extern ext_dart_status_t    			  				 	 dart_state;//飞镖发射状态，飞镖发射后发送 范围所有机器人
extern ext_event_data_t    								   event_data;//场地事件数据，事件改变后发送
extern ext_supply_projectile_action_t         supply_projectile_action;//补给站动作标识 动作改变后发送 己方机器人
extern ext_referee_warning_t									 referee_warning;//裁判警告信息 警告发生后发送
extern ext_dart_remaining_time_t       			 dart_remaining_time;//飞镖发射口倒计时 1Hz 周期发送 己方机器人



extern ext_game_robot_status_t                game_robot_state;//机器人状态数据， 10Hz 周期发送
extern ext_power_heat_data_t                  power_heat_data;//实时功率热量数据， 50Hz 周期发送
extern ext_game_robot_pos_t                   game_robot_pos;//机器人位置数据， 10Hz 发送
extern ext_buff_t                             buff_musk;//机器人增益数据,增益状态改变后发送



extern aerial_robot_energy_t                  aerial_robot_energy;//空中机器人能量状态 10Hz
extern ext_robot_hurt_t                       robot_hurt;//伤害状态数据，伤害发生后发送
extern ext_shoot_data_t										   shoot_data;//实时射击数据，子弹发射后发送
extern ext_bullet_remaining_t								 bullet_remaining;//子弹剩余发射数 10Hz 所有机器人发送
extern ext_rfid_status_t 										 rfid_status;//机器人 RFID 状态 1Hz 单一机器人
extern ext_dart_client_cmd_t                  dart_client_cmd;//飞镖机器人客户端指令数据 10Hz 单一机器人
extern ext_ICRA_buff_debuff_zone_status_t     ICRA_buff_debuff_zone;//人工智能挑战赛加成与惩罚区状态 1Hz 周期发送 己方机器人
extern ext_robot_command_t                      robot_command;      //小地图交互信息标识：0x0303。发送频率：触发时发送。


extern update_event 													 renew;//裁判系统更新消息
extern user_buff_judege											 buff;
extern user_robot_hurt_judege								 hurt;
extern user_robot_RFID_judege 								 rfid;



extern uint8_t User_meta_data[256]__attribute__((at(0x24038000)));
extern uint8_t referee_meta_data[256]__attribute__((at(0x24028000))); //裁判系统接收存储数组
extern uint8_t * Vehicle_Communicate_data __attribute__((at(0x24042000)));

void USART3_IDLE_CALLBACK(void);

#endif //SRC_UART_H
