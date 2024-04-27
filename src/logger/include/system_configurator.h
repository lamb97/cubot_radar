//
// Created by plutoli on 2022/2/4.
//

#ifndef CUBOT_BRAIN_SYSTEM_CONFIGURATOR_H
#define CUBOT_BRAIN_SYSTEM_CONFIGURATOR_H

/**
 * @brief 机器人大脑内核的工作模式
 */
enum class EWorkMode
{
    Manual = 0,                     ///< 手动射击模式
    ShootOne = 1,                   ///< 优先射击1号车模式
    ShootTwo = 2,                   ///< 优先射击2号车模式
    ShootThree = 3,                 ///< 优先射击3号车模式
    ShootFour = 4,                  ///< 优先射击4号车模式
    ShootFive = 5,                  ///< 优先射击5号车模式
    ShootSentry = 6,                ///< 优先射击哨兵模式
    ShootOutpost = 7,               ///< 优先射击前哨站模式
    ShootBase = 8,                  ///< 优先射击基地模式
    AutomaticShoot = 9,             ///< 自动射击模式
    CurvedFireOutpost = 10,         ///< 吊射前哨站模式
    CurvedFireBase = 11,            ///< 吊射基地模式
    ShootSmallBuff = 12,            ///< 击打小符模式
    ShootLargeBuff = 13             ///< 击打大符模式
};

/**
 * @brief 机器人大脑内核的补偿多项式阶数
 */
enum class EPolynomialOrder
{
    One = 1,            ///< 1阶多项式
    Two = 2,            ///< 2阶多项式
    Three = 3,          ///< 3阶多项式
    Four = 4,           ///< 4阶多项式
    Five = 5,           ///< 5阶多项式
    Six = 6,            ///< 6阶多项式
    Seven = 7,          ///< 7阶多项式
    Eight = 8,          ///< 8阶多项式
    Nine = 9            ///< 9阶多项式
};

/**
 * @brief 机器人本体的子弹速度
 */
enum class EBulletVelocity
{
    MPS_10 = 10,        ///< 10米/秒
    MPS_11 = 11,        ///< 11米/秒
    MPS_12 = 12,        ///< 12米/秒
    MPS_13 = 13,        ///< 13米/秒
    MPS_14 = 14,        ///< 14米/秒
    MPS_15 = 15,        ///< 15米/秒
    MPS_16 = 16,        ///< 16米/秒
    MPS_17 = 17,        ///< 17米/秒
    MPS_18 = 18,        ///< 18米/秒
    MPS_19 = 19,        ///< 19米/秒
    MPS_20 = 20,        ///< 20米/秒
    MPS_21 = 21,        ///< 21米/秒
    MPS_22 = 22,        ///< 22米/秒
    MPS_23 = 23,        ///< 23米/秒
    MPS_24 = 24,        ///< 24米/秒
    MPS_25 = 25,        ///< 25米/秒
    MPS_26 = 26,        ///< 26米/秒
    MPS_27 = 27,        ///< 27米/秒
    MPS_28 = 28,        ///< 28米/秒
    MPS_29 = 29,        ///< 29米/秒
    MPS_30 = 30         ///< 30米/秒
};

/**
 * @brief 机器人本体的请求类型
 */
enum class ERequestType
{
    HeartBeat = 1,              ///< 发送心跳包
    SwitchWorkMode = 2,         ///< 切换工作模式
    SwitchBulletVelocity = 3,   ///< 切换子弹速度
    SaveLog = 4,                ///< 保存日志
    RebootApplication = 5,      ///< 重启应用程序
    RebootComputer = 6,          ///< 重启计算机
    PredictorCommond=7          ///<动动测测自瞄
};

/**
 * @brief 系统配置器
 */
class SystemConfigurator
{
public:
    /**
    * @brief 构造函数
    */
    SystemConfigurator() = default;

    /**
     * @brief 析构函数
     */
    ~SystemConfigurator() = default;

    /**
    * @brief 转换机器人大脑内核工作模式
    * @param[in]   input   输入的机器人大脑内核工作模式数值
    * @param[out]  output  转换得到的机器人大脑内核工作模式
    * @return 机器人大脑内核工作模式转换结果\n
    *         -<em>false</em> 转换失败\n
    *         -<em>true</em> 转换成功\n
    * @note 机器人大脑内核工作模式的取值范围为[0,13]的整数，输入数据不在此范围内，则转换失败\n
    *         -<em>0</em> Manual\n
    *         -<em>1</em> ShootOne\n
    *         -<em>2</em> ShootTwo\n
    *         -<em>3</em> ShootThree\n
    *         -<em>4</em> ShootFour\n
    *         -<em>5</em> ShootFive\n
    *         -<em>6</em> ShootSentry\n
    *         -<em>7</em> ShootOutpost\n
    *         -<em>8</em> ShootBase\n
    *         -<em>9</em> AutomaticShoot\n
    *         -<em>10</em> CurvedFireOutpost\n
    *         -<em>11</em> CurvedFireBase\n
    *         -<em>12</em> ShootSmallBuff\n
    *         -<em>13</em> ShootLargeBuff\n
    */
    static bool ConvertToWorkMode(const int &input, EWorkMode *output);

    /**
     * @brief 转换机器人大脑内核的补偿多项式阶数
     * @param[in] input     输入的目标解算器补偿多项式阶数值
     * @param[out] output   转换得到的目标解算器补偿多项式阶数
     * @return 机器人大脑内核的补偿多项式阶数转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 机器人大脑内核的补偿多项式阶数的取值范围为[1,9]的整数，输入数据不在此范围内，则转换失败\n
     *         -<em>1</em> One\n
     *         -<em>2</em> Two\n
     *         -<em>3</em> Three\n
     *         -<em>4</em> Four\n
     *         -<em>5</em> Five\n
     *         -<em>6</em> Six\n
     *         -<em>7</em> Seven\n
     *         -<em>8</em> Eight\n
     *         -<em>9</em> Nine\n
     */
    static bool ConvertToPolynomialOrder(const int &input, EPolynomialOrder *output);

    /**
     * @brief 转换机器人本体的子弹速度
     * @param[in] input     输入的子弹速度数值
     * @param[out] output   转换得到的子弹速度
     * @return 机器人本体的子弹速度转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 机器人本体的子弹速度取值范围为[10, 30]，输入数据不在此范围内，则转换失败\n
     *         -<em>10</em> MPS_10\n
     *         -<em>11</em> MPS_11\n
     *         -<em>12</em> MPS_12\n
     *         -<em>13</em> MPS_13\n
     *         -<em>14</em> MPS_14\n
     *         -<em>15</em> MPS_15\n
     *         -<em>16</em> MPS_16\n
     *         -<em>17</em> MPS_17\n
     *         -<em>18</em> MPS_18\n
     *         -<em>19</em> MPS_19\n
     *         -<em>20</em> MPS_20\n
     *         -<em>21</em> MPS_21\n
     *         -<em>22</em> MPS_22\n
     *         -<em>23</em> MPS_23\n
     *         -<em>24</em> MPS_24\n
     *         -<em>25</em> MPS_25\n
     *         -<em>26</em> MPS_26\n
     *         -<em>27</em> MPS_27\n
     *         -<em>28</em> MPS_28\n
     *         -<em>29</em> MPS_29\n
     *         -<em>30</em> MPS_30\n
     */
    static bool ConvertToBulletVelocity(const int &input, EBulletVelocity *output);

    /**
    * @brief 转换机器人本体的请求类型
    * @param[in]   input   输入的机器人本体请求类型数值
    * @param[out]  output  转换得到的机器人本体请求类型
    * @return 机器人本体请求类型转换结果\n
    *         -<em>false</em> 转换失败\n
    *         -<em>true</em> 转换成功\n
    * @note 机器人本体请求类型的取值范围为[1,5]的整数，输入数据不在此范围内，则转换失败\n
    *         -<em>1</em> HeartBeat\n
    *         -<em>2</em> SwitchWorkMode\n
    *         -<em>3</em> SwitchBulletVelocity\n
    *         -<em>4</em> SaveLog\n
    *         -<em>5</em> RebootApplication\n
    *         -<em>6</em> RebootComputer\n
    */
    static bool ConvertToRequestType(const int &input, ERequestType *output);
};

#endif //CUBOT_BRAIN_SYSTEM_CONFIGURATOR_H