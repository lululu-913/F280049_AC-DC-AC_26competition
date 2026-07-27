/*
 * 2025版 ACDC-3DCAC 控制程序：由 TMS320F28069 移植到 TMS320F280049C。
 *
 * 整流电压/电流双环及逆变纯电压环沿用 2025_acdcac.c；
 * 已按当前拓扑删除功率级DC-DC控制，时钟、GPIO、ADC、中断和ePWM初始化
 * 参考原 project_280049_ACAC/main.c。
 *
 * 为保持 F28069 控制代码含义，逻辑PWM分配如下：
 *   ePWM1/ePWM2/ePWM3：三相逆变 A/B/C 桥臂；
 *   ePWM4：单相整流左桥臂，上下管分别由ePWM4A/ePWM4B驱动；
 *   ePWM5：单相整流右桥臂，上下管分别由ePWM5A/ePWM5B驱动；
 *   ePWM6：未使用，始终强制输出低电平。
 * 该分配不同于原 F280049 AC-AC 工程的“ePWM1/2前级、ePWM4/5/6后级”分组；
 * 接通功率电路前必须核对PCB门极驱动接线。
 */

#include "F28x_Project.h"                                                   // 包含F28004x寄存器、系统及外设定义
#include "driverlib.h"                                                      // 包含F280049 DriverLib接口
#include "math.h"                                                           // 提供单精度三角函数、平方根和绝对值函数
#include "oled.h"                                                           // 包含OLED显示驱动接口

//********** 函数声明 **********//
__interrupt void adcA1ISR(void);                                            // 20kHz ADC中断与全部实时控制算法

void InitADC(void);                                                         // 初始化ADCA、ADCB与ADCC模拟模块
void InitADCSOC(void);                                                      // 配置ADC通道、采样窗、触发源和中断
void Init_KEY(void);                                                        // 配置目标板六个按键输入
char KEY_Scan(char mode);                                                   // 读取六个低电平有效按键
void KEY_Control(int key);                                                  // 根据按键修改模式或参考值
void OLED_output(void);                                                     // 刷新设定值和测量值显示

void InitEPWM(void);                                                        // 冻结时基并统一配置所有ePWM模块
void EPWM1_Init(void);                                                      // 初始化ePWM1
void EPWM2_Init(void);                                                      // 初始化ePWM2
void EPWM3_Init(void);                                                      // 初始化ePWM3及SOCA触发
void EPWM4_Init(void);                                                      // 初始化ePWM4
void EPWM5_Init(void);                                                      // 初始化ePWM5
void EPWM6_Init(void);                                                      // 初始化未使用的ePWM6

void PLL1(float UI);                                                        // SOGI-PLL输入电压锁相计算
void PID1_Init(void);                                                       // 初始化母线电压外环参数
void PID2_Init(void);                                                       // 初始化输入电流PI控制器
void PIDa_Init(void);                                                       // 初始化逆变输出电压控制参数
float PID1_Cal(float u);                                                    // 计算母线电压外环增量
float PID2_Cal(float u);                                                    // 计算输入电流PI控制输出
float PIDa_Cal(float u);                                                    // 计算逆变调制分母增量

static void PWM_ForceAllLow(void);                                          // 通过Trip Zone立即关断全部ePWM输出
static void PWM_TripInverter(void);                                         // 仅通过Trip Zone关断逆变侧ePWM1～3
static void PWM_TripRectifier(void);                                        // 仅通过Trip Zone关断整流侧ePWM4/5
static void PWM_ReleaseInverter(void);                                      // 按预装阶段释放逆变侧ePWM1～3
static void PWM_ReleaseRectifier(void);                                     // 按预装阶段释放整流侧ePWM4/5
static void SVPWM_Calculate(float phase_a, float phase_b, float phase_c,     // 计算保持现有相序和基波幅值的SVPWM三相占空比
                            float modulation_a, float modulation_b,
                            float modulation_c, float *duty_a,
                            float *duty_b, float *duty_c);
static void PhaseBalance_Calculate(float ua_rms, float ub_rms, float uc_rms,// 计算零和、同比例限幅的三相幅值补偿
                                   float *trim_a, float *trim_b,
                                   float *trim_c);

//********** 按键变量（沿用F280049工程板引脚）**********//
#define KEY_H1          (GpioDataRegs.GPADAT.bit.GPIO27)                    // 读取对应目标板按键GPIO电平
#define KEY_H2          (GpioDataRegs.GPADAT.bit.GPIO25)                    // 读取对应目标板按键GPIO电平
#define KEY_H3          (GpioDataRegs.GPADAT.bit.GPIO17)                    // 读取对应目标板按键GPIO电平
#define KEY_H4          (GpioDataRegs.GPADAT.bit.GPIO26)                    // 读取对应目标板按键GPIO电平
#define KEY_H5          (GpioDataRegs.GPADAT.bit.GPIO16)                    // 读取目标板KEY5对应的GPIO16电平
#define KEY_H6          (GpioDataRegs.GPBDAT.bit.GPIO39)                    // 读取目标板KEY6对应的GPIO39电平

#define KEY1_PRESS  1                                                       // 定义对应按键返回码
#define KEY2_PRESS  2                                                       // 定义对应按键返回码
#define KEY3_PRESS  3                                                       // 定义对应按键返回码
#define KEY4_PRESS  4                                                       // 定义对应按键返回码
#define KEY5_PRESS  5                                                       // 定义KEY5按键返回码
#define KEY6_PRESS  6                                                       // 定义KEY6按键返回码
#define KEY_UNPRESS 0                                                       // 定义无按键返回码
int key = 0;                                                                // 保存当前按键扫描结果
int N_key = 0;                                                              // KEY3～KEY6原500ms扫描分频计数器
int N_freq_key = 0;                                                         // KEY1/KEY2频率调节250ms重复分频计数器

//********** Flash运行段变量 **********//
#ifdef _FLASH                                                               // 仅在Flash构建时启用RAM函数段复制
extern Uint16 RamfuncsLoadStart;                                            // Flash中RAM函数段的装载起始符号
extern Uint16 RamfuncsRunStart;                                             // RAM中RAM函数段的运行起始符号
extern Uint16 RamfuncsLoadSize;                                             // RAM函数段复制长度符号
#endif                                                                      // 结束条件编译

//********** PID数据结构与实例 **********//
typedef struct {                                                            // 定义通用PID参数与运行状态结构体
    float ref;                                                               // 控制器参考值
    float Xin;                                                               // 控制器测量输入值
    float Err;                                                               // 当前控制误差
    float Err_last;                                                          // 上一拍控制误差
    float Kp, Ki;                                                            // 比例和积分系数
    float result;                                                            // 当前控制器输出
    float Integral;                                                          // 积分累加状态
} pidsettings;                                                              // 结束PID结构体定义

pidsettings pid1;                                                           // 母线电压外环控制器
pidsettings pid2;                                                           // 输入电流比例控制器
pidsettings pida;                                                           // 三相逆变输出电压控制器

//********** 基础控制变量 **********//
// F280049系统时钟为100MHz，周期值2500可保持原20kHz开关频率
// 与ADC采样频率，使Ts及所有离散控制器系数继续有效。
#define EPWM_TIMER_TBPRD  2500                                              // 100MHz上下计数时对应20kHz PWM
#define pi 3.1415926f                                                       // 定义单精度圆周率常量
#define OUTPUT_FREQ_MIN_HZ 45U                                              // KEY2允许设置的三相逆变输出频率下限
#define OUTPUT_FREQ_MAX_HZ 505U                                             // KEY1允许设置的三相逆变输出频率上限
#define OUTPUT_FREQ_STEP_HZ 1U                                              // KEY1/KEY2每次把三相逆变输出频率调整1Hz
#define FREQ_KEY_REPEAT_ISR_DIV 5000                                        // KEY1/KEY2按20kHz ISR分频，实现250ms长按重复
#define GENERAL_KEY_SCAN_ISR_DIV 10000                                      // KEY3～KEY6保持原500ms扫描与重复周期
#define INPUT_CURRENT_PK_MAX 6.00f                                          // 满功率时限制整流输入电流参考峰值为6.00A
#define INPUT_OVERCURRENT_LIMIT 9.0f                                        // 设置整流输入瞬时软件过流保护阈值为9A
#define BUS_OVERVOLTAGE_LIMIT 65.0f                                         // 47.5V默认母线以上保留12.5V瞬时软件过压裕量
#define BUS_CURRENT_SLEW_UP_STEP 0.010f                                     // 每10ms允许外环电流幅值增加0.010A，实现约1.0A/s软启动
#define BUS_CURRENT_SLEW_DOWN_STEP 0.020f                                   // 每10ms允许外环电流幅值减小0.020A，实现约2.0A/s快速降流
#define BUS_CONTROL_MIN_VOLTAGE 5.0f                                        // 母线电压归一化分母下限，避免启动时除数过小
#define CURRENT_CTRL_VOLTAGE_LIMIT 15.0f                                    // 限制电流环电感补偿电压，抑制占空比突变
#define RECTIFIER_CURRENT_KP 8.0f                                           // 输入电流环比例增益
#define RECTIFIER_CURRENT_KI 0.01f                                          // 输入电流环保守积分增益
#define RECTIFIER_CURRENT_INTEGRAL_LIMIT 30.0f                              // 积分状态限幅，对应最大正负0.3V积分补偿
#define RECTIFIER_FEEDFORWARD_GAIN 0.75f                                     // 仿照单极性示例叠加0.5倍输入电压前馈
#define RECTIFIER_MODULATION_LIMIT 0.95f                                    // 将单极性调制量Di限制在正负0.95以内
#define BUS_REF_HEADROOM_VOLTAGE 5.0f                                       // 母线目标至少高于输入电压峰值5V
#define INVERTER_SOFT_GAIN_UP_STEP 0.02f                                    // 每10ms增加0.02，约0.5s完成逆变软启动
#define INVERTER_SOFT_GAIN_DOWN_STEP 0.05f                                  // 每10ms减小0.05，约0.2s完成逆变软关断
#define OUTPUT_COMMON_IIR_ALPHA 0.00624f                                    // 20kHz下约20Hz公共三相电压平方滤波
#define OUTPUT_PHASE_IIR_ALPHA 0.000628f                                    // 20kHz下约2Hz单相电压平方滤波，抑制100Hz分量
#define BUS_FEEDFORWARD_IIR_ALPHA 0.086f                                    // 20kHz下约300Hz母线前馈滤波，跟踪100Hz纹波
#define BUS_FEEDFORWARD_GAIN_MIN 0.85f                                      // 母线前馈最小增益，限制采样异常造成的突降
#define BUS_FEEDFORWARD_GAIN_MAX 1.15f                                      // 母线前馈最大增益，限制欠压时调制度突升
#define INVERTER_MODULATION_LIMIT 0.56f                                     // SVPWM公共调制度限幅，含1%平衡补偿仍在线性区
#define PHASE_BALANCE_KP 0.01f                                              // 每相有效值误差1V对应1%幅值补偿
#define PHASE_BALANCE_TRIM_LIMIT 0.01f                                      // 每相独立补偿限制在正负1%
#define PHASE_BALANCE_ENABLE_GAIN 0.95f                                     // 软启动达到95%后才允许相间平衡控制
#define PHASE_BALANCE_SETTLE_WINDOWS 50U                                    // 满幅后等待50个10ms窗口再启用，即500ms
#define OLED_REFRESH_DIVIDER 1000                                           // 20kHz中断分频1000次，每50ms请求刷新一行OLED

float U_REF = 24.0f;                                                        // 满功率输入交流额定有效值为24V
volatile float U_BUS_REF = 47.5f;                                           // 整流输出直流母线默认目标电压为47.5V
volatile float U_OUT_REF = 17.3205f;                                        // 三相相电压目标为17.3205Vrms，对应线电压30Vrms
volatile Uint16 output_freq_hz = 50U;                                       // 三相逆变输出频率命令，KEY1/KEY2在45～505Hz内调节

volatile int rectifier_enable = 0;                                          // 整流独立模式命令：0被动整流、1主动整流
volatile int inverter_enable = 0;                                           // 逆变独立模式命令：0请求关断、1请求运行
int rectifier_enable_last = 0;                                              // 保存上一拍整流命令以检测KEY3切换边沿
int inverter_enable_last = 0;                                               // 保存上一拍逆变命令以检测KEY6切换边沿
volatile int tag = 0;                                                       // 仅供在线观察的组合状态位：bit1整流、bit0逆变
int inverter_pwm_start_stage = 0;                                           // 逆变PWM启动阶段：0关断、1预装、2待释放、3正常运行
int rectifier_pwm_start_stage = 0;                                          // 整流PWM启动阶段：0关断、1预装、2待释放、3正常运行
int inverter_soft_stop_active = 0;                                          // 逆变软关断状态：1时逐步把调制幅值降为零
float inverter_soft_gain = 0.0f;                                            // 逆变三相正弦幅值系数：0完全降幅、1正常输出
volatile int rectifier_fault = 0;                                           // 整流局部故障锁存，不直接关断逆变侧
volatile int system_fault = 0;                                              // 预留的全局故障锁存；母线过压不再置位该标志
int oled_row = 0;                                                           // 保存下一次需要刷新的OLED行号
int N_oled = 0;                                                             // OLED刷新请求的20kHz分频计数器
volatile int oled_refresh_request = 1;                                      // OLED按行刷新请求标志，上电后先请求刷新第0行
int N = 200;                                                                // 10ms统计窗口长度，与三相输出频率相互独立
float sum1 = 0;                                                             // 保存母线平均值累加
int N_c1 = 0, N_c2 = 0;                                                     // 分别作为整流母线窗口和逆变输出窗口计数器
float M = 1.94f;                                                            // 47.5V母线输出30V线电压时的额定SVPWM调制分母
volatile int flag = 0;                                                      // 保护原因标志，由控制ISR写入并由主循环显示

float U_oa = 0, U_ob = 0, U_oc = 0;                                         // 重构得到的三相输出相电压
float U_oab = 0, U_obc = 0;                                                 // 三相输出线电压变量
float theta_a = 0, theta_b = 0, theta_c = 0;                                // 三相SPWM参考相位
float Da = 0.4, Db = 0.4, Dc = 0.4;                                         // 保留的三相占空比变量
float U_bus = 0;                                                            // 直流母线瞬时采样值
float U_av = 0;                                                             // 由三相相电压计算的瞬时等效有效值
float output_common_sq_sample = 0.0f;                                       // 三相相电压平方平均的瞬时样本
float output_common_sq_f = 0.0f;                                            // 三相公共有效值平方的快速IIR状态
float output_phase_a_sq_f = 0.0f;                                           // A相有效值平方的慢速IIR状态
float output_phase_b_sq_f = 0.0f;                                           // B相有效值平方的慢速IIR状态
float output_phase_c_sq_f = 0.0f;                                           // C相有效值平方的慢速IIR状态
volatile float U_oa_rms = 0.0f;                                             // 滤波后的A相有效值
volatile float U_ob_rms = 0.0f;                                             // 滤波后的B相有效值
volatile float U_oc_rms = 0.0f;                                             // 滤波后的C相有效值
float U_phase_avg = 0.0f;                                                   // 三相有效值平均，用作独立补偿公共基准
float U_bus_ff = 47.5f;                                                     // 逆变瞬时母线前馈滤波状态
float bus_feedforward_gain = 1.0f;                                          // 母线参考值与实时母线值之比
float inverter_modulation = 0.0f;                                           // 经母线前馈和总限幅后的公共调制度
float modulation_a = 0.0f, modulation_b = 0.0f, modulation_c = 0.0f;        // 叠加相间平衡补偿后的三相调制度
float phase_trim_a = 0.0f, phase_trim_b = 0.0f, phase_trim_c = 0.0f;         // 三相零和幅值补偿系数
Uint16 phase_balance_settle_windows = 0U;                                   // 满幅运行后的平衡控制等待计数

float U_in = 0;                                                             // 输入电压
float I_in = 0;                                                             // 输入电流
volatile float U_bus_rms = 0;                                               // 直流母线10ms平均值，由控制ISR更新并由主循环显示
volatile float U_out_rms = 0;                                               // 三相交流输出窗口有效值，由控制ISR更新并由主循环显示
float pid_out = 0;                                                          // 母线电压外环累计得到的输入电流正峰值
float pll_out = 0;                                                          // 乘入PLL相位后的电流参考值
float D1 = 0.5f;                                                            // 整流左桥臂占空比，右桥臂使用1-D1
float middle = 0;                                                           // 输入电流内环补偿量及桥臂参考电压中间变量
volatile float I_ref_mag = 0.0f;                                            // 母线电压外环给出的带符号电流峰值，保留原有负幅值约定
float i_l_ref = 0.0f;                                                       // 整流电感与输入电压同相的瞬时电流参考
float err_i = 0.0f;                                                         // 比例电流环的瞬时跟踪误差
float u_i_out = 0.0f;                                                       // 比例电流控制器输出电压补偿量
float i_ctrl = 0.0f;                                                        // 电流补偿与输入电压前馈合成的桥侧电压指令
volatile float Di = 0.0f;                                                   // ISR更新的单极性整流调制量，后台OLED读取其快照
float v_dc_bus = 10.0f;                                                     // 单极性调制归一化使用的实时母线电压

//************* 锁相环参数 *************//
float W0 = 314.159f;                                                        // 50Hz额定角频率100π
float W1 = 314.159f;                                                        // PLL估算角频率，初值为50Hz
float W1_pi = 0.0f;                                                        // PLL估算频率的Hz显示量
float Ts = 0.00005f;                                                        // PLL采样周期为50微秒，对应20kHz控制中断
float k = 1.0f;                                                             // SOGI传递函数比例系数
float x = 0.0f;                                                             // 输入电压与SOGI α轴输出的瞬时误差
float Ua = 0.0f;                                                            // SOGI生成的α轴正交分量
float Ub = 0.0f;                                                            // SOGI生成的β轴正交分量
float Ud = 0.0f;                                                            // PLL同步旋转坐标系d轴分量
float Uq = 0.0f;                                                            // PLL同步旋转坐标系q轴分量
float theta = 0.0f;                                                         // PLL输出电角度
float kp = 100.0f;                                                          // 采用已核对版本的PLL比例参数
float ki = 10.0f;                                                           // 采用已核对版本的PLL积分参数
float err = 0.0f;                                                           // PLL当前q轴误差
float last_err = 0.0f;                                                      // PLL上一拍q轴误差
float result = 0.0f;                                                        // PLL本拍角频率修正增量

//******************* 主函数 *******************//
void main(void)                                                             // 程序入口，完成F280049外设和控制器初始化
{
#ifdef _FLASH                                                               // 仅在Flash构建时启用RAM函数段复制
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize); // 把需在RAM执行的函数段从Flash复制到RAM
#endif                                                                      // 结束条件编译

    InitSysCtrl();                                                          // 按目标工程配置100MHz系统时钟及外设时钟
    InitGpio();                                                             // 恢复GPIO默认配置

    // F280049复位后GPIO配置可能处于锁定状态，配置复用功能前先解锁。
    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFFUL);                       // 解锁对应GPIO端口配置寄存器
    GPIO_unlockPortConfig(GPIO_PORT_B, 0xFFFFFFFFUL);                       // 解锁对应GPIO端口配置寄存器
    GPIO_unlockPortConfig(GPIO_PORT_H, 0xFFFFFFFFUL);                       // 解锁对应GPIO端口配置寄存器

    EALLOW;                                                                 // 允许访问受保护寄存器
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // 复用为ePWM1A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // 复用为ePWM1B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // 复用为ePWM2A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // 复用为ePWM2B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // 复用为ePWM3A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // 复用为ePWM3B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // 复用为ePWM4A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // 复用为ePWM4B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // 复用为ePWM5A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // 复用为ePWM5B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;  // GPIO10复用为ePWM6A（未使用）
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;  // GPIO11复用为ePWM6B（未使用）
    AnalogSubsysRegs.DCDCCTL.bit.DCDCEN = 0;                                // 关闭芯片内部DC-DC以适配外部供电
    EDIS;                                                                   // 重新禁止访问受保护寄存器

    DINT;                                                                   // 初始化期间关闭CPU全局中断
    InitPieCtrl();                                                          // 初始化PIE中断控制器
    IER = 0x0000;                                                           // 清除CPU中断使能寄存器
    IFR = 0x0000;                                                           // 清除CPU中断标志寄存器
    InitPieVectTable();                                                     // 初始化PIE中断向量表

    EALLOW;                                                                 // 允许访问受保护寄存器
    PieVectTable.ADCA1_INT = &adcA1ISR;                                     // 将ADCA1中断向量绑定到控制ISR
    EDIS;                                                                   // 重新禁止访问受保护寄存器

    Init_KEY();                                                             // 配置目标板按键GPIO
    InitADC();                                                              // 上电并配置ADCA与ADCB
    InitEPWM();                                                             // 冻结时基并配置六组ePWM
    InitADCSOC();                                                           // 配置ADC采样通道、触发源和中断源
    PID1_Init();                                                            // 初始化母线电压外环
    PID2_Init();                                                            // 初始化输入电流PI控制器
    PIDa_Init();                                                            // 初始化三相输出电压调节器
    OLED_Init();                                                            // 初始化OLED显示模块
    OLED_ShowString(0, 0, "R  I  F  M      ");                              // 第0行固定显示整流、逆变、故障和调制分母
    OLED_ShowString(0, 1, "BR      B       ");                              // 固定标签仅在上电初始化时写入第1行
    OLED_ShowString(0, 2, "OR      O       ");                              // 固定标签仅在上电初始化时写入第2行
    OLED_ShowString(0, 3, "F       DI      ");                              // 第3行固定显示逆变频率命令和带符号的整流调制量

    IER |= M_INT1;                                                          // 允许CPU响应PIE第1组中断
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;                                      // 使能PIE第1组第1路ADCA中断

    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;                                   // 统一释放全部ePWM时基并同步启动
    EDIS;                                                                   // 重新禁止访问受保护寄存器

    EINT;                                                                   // 开启CPU可屏蔽中断
    ERTM;                                                                   // 开启实时调试中断响应

    for(;;)                                                                 // 进入永久后台循环
    {
        if(oled_refresh_request != 0)                                       // 收到50ms分频产生的OLED刷新请求
        {
            oled_refresh_request = 0;                                       // 先清除请求，避免连续占用后台执行时间
            OLED_output();                                                  // 每次只刷新一行固定变量界面
        }
        if(U_BUS_REF > 55.0f) U_BUS_REF = 55.0f;                            // 参考上限与60V过压保护之间保留5V动态裕量
        if(U_BUS_REF < (1.4142f * U_REF + BUS_REF_HEADROOM_VOLTAGE))         // 保证升压整流母线目标高于输入电压峰值
            U_BUS_REF = 1.4142f * U_REF + BUS_REF_HEADROOM_VOLTAGE;          // 保留约5V调制裕量，避免无法正常控流
        if(U_OUT_REF > 30) U_OUT_REF = 30;                                  // 限制三相输出目标电压上限
        if(U_OUT_REF < 1) U_OUT_REF = 1;                                    // 限制三相输出目标电压下限
        // 控制器参考值在对应100Hz控制窗口内读取，避免主循环直接改写ISR内部状态。
    }
}

//******************* ADC模块初始化 *******************//
void InitADC(void)                                                          // 初始化ADCA、ADCB与ADCC模拟模块
{
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCA);                         // 完整复位ADCA并由驱动库重新装载芯片ADC校准值
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCB);                         // 完整复位ADCB并由驱动库重新装载芯片ADC校准值
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCC);                         // 完整复位ADCC，保证与ADCB共享的模拟参考网络状态一致

    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);     // 配置对应ADC使用内部3.3V参考源
    EALLOW;                                                                 // 允许访问受保护寄存器
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;                                      // 100MHz系统时钟四分频，ADCCLK约25MHz
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;                                   // 在转换结果锁存后产生ADC中断脉冲
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;                                      // 给对应ADC模拟模块上电
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    DELAY_US(1000);                                                         // 等待ADC模拟电路上电稳定

    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);     // 配置对应ADC使用内部3.3V参考源
    EALLOW;                                                                 // 允许访问受保护寄存器
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;                                      // 将ADC时钟配置为约25MHz
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;                                   // 在转换结果锁存后产生ADC中断脉冲
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;                                      // 给对应ADC模拟模块上电
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    DELAY_US(1000);                                                         // 等待ADC模拟电路上电稳定

    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);     // 将共享参考网络上的ADCC也配置为内部3.3V参考
    EALLOW;                                                                 // 允许访问受保护寄存器
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;                                      // 将ADCC时钟配置为约25MHz
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;                                   // 在转换结果锁存后产生ADC中断脉冲
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;                                      // 给ADCC模拟模块上电以稳定ADCB/ADCC共享参考网络
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    DELAY_US(1000);                                                         // 等待ADCC及共享模拟参考网络上电稳定
}

void InitADCSOC(void)                                                       // 配置ADC通道、采样窗、触发源和中断
{
    EALLOW;                                                                 // 允许访问受保护寄存器

    AdcaRegs.ADCBURSTCTL.all = 0;                                           // 关闭ADCA突发转换，防止非预期SOC连续触发
    AdcaRegs.ADCSOCPRICTL.all = 0;                                          // 恢复ADCA默认轮询优先级
    AdcaRegs.ADCINTSOCSEL1.all = 0;                                         // 禁止ADCA ADCINT1～4再次触发SOC0～7
    AdcaRegs.ADCINTSOCSEL2.all = 0;                                         // 禁止ADCA ADCINT1～4再次触发SOC8～15
    AdcaRegs.ADCSOCOVFCLR1.all = 0xFFFF;                                    // 清除ADCA全部SOC触发溢出粘滞标志
    AdcaRegs.ADCINTOVFCLR.all = 0x000F;                                     // 清除ADCA全部ADC中断溢出标志
    AdcaRegs.ADCINTFLGCLR.all = 0x000F;                                     // 清除ADCA全部ADC中断标志

    AdcbRegs.ADCBURSTCTL.all = 0;                                           // 关闭ADCB突发转换，确保B0仅由配置的SOCA触发
    AdcbRegs.ADCSOCPRICTL.all = 0;                                          // 恢复ADCB默认轮询优先级
    AdcbRegs.ADCINTSOCSEL1.all = 0;                                         // 禁止ADCB ADCINT1～4再次触发SOC0～7
    AdcbRegs.ADCINTSOCSEL2.all = 0;                                         // 禁止ADCB ADCINT1～4再次触发SOC8～15
    AdcbRegs.ADCSOCOVFCLR1.all = 0xFFFF;                                    // 清除ADCB全部SOC触发溢出粘滞标志
    AdcbRegs.ADCINTOVFCLR.all = 0x000F;                                     // 清除ADCB全部ADC中断溢出标志
    AdcbRegs.ADCINTFLGCLR.all = 0x000F;                                     // 清除ADCB全部ADC中断标志

    // 将F28069的SOC/通道关系分别映射到F280049的ADCA和ADCB。
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;                                      // SOC0选择ADCINA0，采集输入电压U_in
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9;                                      // 设置ADC采样保持窗口为10个SYSCLK
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 9;                                    // 选择ePWM3 SOCA作为硬件触发源

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;                                      // SOC1选择ADCINA1，采集母线电压U_bus
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9;                                      // 设置ADC采样保持窗口为10个SYSCLK
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 9;                                    // 选择ePWM3 SOCA作为硬件触发源

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;                                      // SOC2选择ADCINA2，采集输出线电压U_oab
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 9;                                      // 设置ADC采样保持窗口为10个SYSCLK
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 9;                                    // 选择ePWM3 SOCA作为硬件触发源

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;                                       // SOC3选择ADCINA3，采集输出线电压U_obc
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 9;                                      // 设置ADC采样保持窗口为10个SYSCLK
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 9;                                    // 选择ePWM3 SOCA作为硬件触发源

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;                                      // SOC0选择ADCINB0，采集输入电流I_in
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 9;                                      // 设置ADC采样保持窗口为10个SYSCLK
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 9;                                    // 选择ePWM3 SOCA作为硬件触发源

    // 逆变侧采用纯电压环，因此不再配置ADCB1～3三相电流采样。
    // EOC3是最后一个使用的ADCA结果，此时并行转换的ADCB RESULT0早已完成。
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;                                  // 选择ADCA EOC3作为ADCINT1中断源
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;                                    // 使能ADCA INT1中断输出
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                                  // 清除ADCA INT1标志以允许下一次中断

    EDIS;                                                                   // 重新禁止访问受保护寄存器
}

//******************* ADC中断（20kHz控制核心）*******************//
__interrupt void adcA1ISR(void)                                             // 20kHz ADC中断与全部实时控制算法
{
    U_in = ((float)AdcaResultRegs.ADCRESULT0 - 2066.2f) / 36.443f;            // 按y=36.443x+2066.2将ADCINA0原始值反算为输入电压
    U_bus = ((float)AdcaResultRegs.ADCRESULT1 - 2067.9f) / 26.268f;           // 按y=26.268x+2067.9将ADCINA1原始值反算为母线电压
    U_oab = ((float)AdcaResultRegs.ADCRESULT2 - 2066.7f) / 36.353f;           // 按y=36.353x+2066.7将ADCINA2原始值反算为输出线电压Uab
    U_obc = ((float)AdcaResultRegs.ADCRESULT3 - 2067.9f) / 36.169f;           // 按y=36.169x+2067.9将ADCINA3原始值反算为输出线电压Ubc
    I_in = ((float)AdcbResultRegs.ADCRESULT0 - 2077.3f) / 164.61f;            // 按y=164.61x+2077.3将ADCINB0原始值反算为输入电流

    if((system_fault == 0) &&                                               // 已有全局故障时不再用局部故障覆盖OLED故障码
       (fabsf(I_in) > INPUT_OVERCURRENT_LIMIT))                             // 检测输入瞬时电流是否超过9A软件保护阈值
    {
        rectifier_enable = 0;                                               // 整流过流只撤销主动整流命令，不直接改变逆变运行命令
        rectifier_fault = 1;                                                // 锁存整流局部故障，等待KEY3重新确认启动
        rectifier_pwm_start_stage = 0;                                      // 清零整流PWM启动阶段
        PWM_TripRectifier();                                                // 立即Trip ePWM4/5并退回MOSFET体二极管被动整流
        flag = 1;                                                           // 记录输入过流故障
    }
    if(U_bus >= BUS_OVERVOLTAGE_LIMIT)                                      // 每个控制周期检测母线是否超过65V软件保护阈值
    {
        rectifier_enable = 0;                                               // 母线过压时撤销主动整流运行命令
        rectifier_fault = 1;                                                // 将母线过压锁存为整流侧局部故障
        rectifier_pwm_start_stage = 0;                                      // 清零整流PWM启动阶段
        PWM_TripRectifier();                                                // 只Trip ePWM4/5，保持逆变ePWM1～3继续运行
        flag = 2;                                                           // 记录母线过压故障并由OLED显示F2
    }
    if(system_fault != 0)                                                   // 全局故障锁存期间始终保持全部功率级关断
    {
        rectifier_enable = 0;                                               // 禁止全局故障期间重新运行主动整流
        inverter_enable = 0;                                                // 禁止全局故障期间重新运行逆变
        PWM_ForceAllLow();                                                   // 每拍保持全部One-Shot Trip锁存
    }

    PLL1(U_in);                                                             // 停机时也持续锁相，使启动前PLL已经跟踪输入电压

    U_ob = (U_obc - U_oab) * 0.3333f;                                       // 由Uab、Ubc重构B相相电压
    U_oa = U_ob + U_oab;                                                    // 由B相相电压和Uab重构A相相电压
    U_oc = -(U_ob + U_oa);                                                  // 利用三相电压和为零重构C相相电压
    U_av = sqrtf((U_oa * U_oa + U_ob * U_ob + U_oc * U_oc) * 0.3333f);      // 计算三相瞬时等效有效值
    output_common_sq_sample = (U_oa * U_oa + U_ob * U_ob + U_oc * U_oc) *   // 平衡三相时该量不含2倍输出频率脉动
                              0.3333333f;
    output_common_sq_f += OUTPUT_COMMON_IIR_ALPHA *                         // 快速滤波公共三相幅值供总电压环使用
                          (output_common_sq_sample - output_common_sq_f);
    output_phase_a_sq_f += OUTPUT_PHASE_IIR_ALPHA *                         // 慢速滤波各相平方以抑制单相2倍频脉动
                           (U_oa * U_oa - output_phase_a_sq_f);
    output_phase_b_sq_f += OUTPUT_PHASE_IIR_ALPHA *
                           (U_ob * U_ob - output_phase_b_sq_f);
    output_phase_c_sq_f += OUTPUT_PHASE_IIR_ALPHA *
                           (U_oc * U_oc - output_phase_c_sq_f);

    if(U_bus > BUS_CONTROL_MIN_VOLTAGE)                                     // 母线有效时跟踪瞬时母线纹波
    {
        U_bus_ff += BUS_FEEDFORWARD_IIR_ALPHA * (U_bus - U_bus_ff);
    }
    else                                                                    // 欠压或采样异常时回到参考值，避免除零与突增
    {
        U_bus_ff = U_BUS_REF;
    }

    if(rectifier_enable != rectifier_enable_last)                           // 检测KEY3产生的主动整流独立模式切换
    {
        if(rectifier_enable != 0)                                           // KEY3请求启动主动整流
        {
            PWM_TripRectifier();                                            // 启动前保持ePWM4/5的Trip Zone关断锁存
            rectifier_pwm_start_stage = 0;                                  // 等待逆变ePWM1～3先进入正常运行阶段再启动整流
            pid_out = 0.0f;                                                 // 从零开始建立外环电流幅值，避免沿用开环测试给定
            I_ref_mag = 0.0f;                                               // 整流启动前把瞬时电流参考幅值清零
            pid1.Err = 0.0f;                                                // 清零母线外环当前误差
            pid1.Err_last = 0.0f;                                           // 清零母线外环上一拍误差，防止切换冲击
            pid1.result = 0.0f;                                             // 清零母线外环本拍电流增量
            rectifier_fault = 0;                                            // 用户重新开启时清除整流局部故障锁存
            if(flag == 1) flag = 0;                                         // 仅清除对应的整流过流显示
        }
        else                                                                // KEY3请求停止主动整流
        {
            PWM_TripRectifier();                                            // 只关断ePWM4/5，逆变侧继续根据自身命令和母线运行
            rectifier_pwm_start_stage = 0;                                  // 下次启动必须重新完成CMPA预装流程
            pid_out = 0.0f;                                                 // 清零母线外环累计电流幅值
            I_ref_mag = 0.0f;                                               // 清零整流瞬时电流参考幅值
        }
    }
    rectifier_enable_last = rectifier_enable;                               // 保存当前整流命令供下一拍检测边沿

    if(inverter_enable != inverter_enable_last)                             // 检测KEY6产生的逆变独立模式切换
    {
        if(inverter_enable != 0)                                            // KEY6请求启动逆变
        {
            inverter_soft_stop_active = 0;                                  // 新的运行请求取消普通软关断状态
        }
        else if((inverter_pwm_start_stage == 3) && (inverter_soft_gain > 0.0f)) // 运行中的逆变收到KEY6关闭命令
        {
            inverter_soft_stop_active = 1;                                  // 保持PWM工作并开始按10ms节拍降低输出幅值
        }
        else                                                                // 尚未正常输出时收到关闭命令
        {
            PWM_TripInverter();                                             // 无需等待软斜坡，直接保持逆变Trip关断
            inverter_pwm_start_stage = 0;                                   // 清零逆变启动阶段
            inverter_soft_gain = 0.0f;                                      // 清零逆变输出幅值
        }
    }
    inverter_enable_last = inverter_enable;                                 // 保存当前逆变命令供下一拍检测边沿

    if((inverter_enable != 0) && (inverter_pwm_start_stage == 0) &&          // 逆变命令开启且PWM尚未启动
       (system_fault == 0))                                                 // 不再使用母线欠压作为逆变启动限制
    {
        PWM_TripInverter();                                                 // 自动重启前再次确认ePWM1～3保持Trip关断
        inverter_pwm_start_stage = 1;                                       // 进入逆变CMPA预装阶段
        inverter_soft_gain = 0.0f;                                          // 每次启动均从零输出幅值开始
        inverter_soft_stop_active = 0;                                      // 清除旧软关断状态
        pida.Err = 0.0f;                                                    // 清零逆变电压环当前误差
        pida.Err_last = 0.0f;                                               // 清零逆变电压环上一拍误差
        pida.result = 0.0f;                                                 // 清零逆变电压环本拍增量
    }

    if((rectifier_enable != 0) && (rectifier_fault == 0) &&                 // 存在主动整流运行请求且整流侧无故障
       (system_fault == 0) && (rectifier_pwm_start_stage == 0) &&           // 整流PWM尚未开始启动
       (inverter_pwm_start_stage == 3))                                     // 必须确认逆变ePWM1～3已经先安全释放
    {
        PWM_TripRectifier();                                                // 预装整流比较值前继续保持ePWM4/5关断
        rectifier_pwm_start_stage = 1;                                      // 逆变已运行后才允许进入整流PWM预装阶段
    }

    N_c1++;                                                                 // 独立累加母线10ms平均值窗口，不再依赖逆变是否运行
    sum1 += U_bus;                                                          // 累加母线瞬时采样供整流外环使用
    if(N_c1 >= N)                                                           // 母线统计窗口达到200个20kHz采样点
    {
        N_c1 = 0;                                                           // 清零母线统计窗口计数
        U_bus_rms = sum1 / N;                                               // 更新母线10ms平均值
        sum1 = 0.0f;                                                        // 清零母线累加器
        if((rectifier_enable != 0) && (rectifier_fault == 0) &&             // 仅主动整流正常请求下更新母线电压外环
           (system_fault == 0))
        {
            pid1.ref = U_BUS_REF;                                           // 读取独立的母线电压目标值
            pid1.result = PID1_Cal(U_bus_rms);                              // 以100Hz更新母线外环电流幅值增量
            if(pid1.result > BUS_CURRENT_SLEW_UP_STEP)                      // 限制正向电流幅值变化率
                pid1.result = BUS_CURRENT_SLEW_UP_STEP;                     // 启动或输入电压降低时约以1.0A/s增加
            else if(pid1.result < -BUS_CURRENT_SLEW_DOWN_STEP)              // 单独限制反向电流幅值变化率
                pid1.result = -BUS_CURRENT_SLEW_DOWN_STEP;                  // 负载减小时约以2.0A/s降低
            pid_out += pid1.result;                                         // 累计得到有功输入电流峰值参考
            if(pid_out > INPUT_CURRENT_PK_MAX) pid_out = INPUT_CURRENT_PK_MAX; // 将母线外环电流峰值参考限制在6A
            if(pid_out < 0.0f) pid_out = 0.0f;                              // PFC仅允许交流侧向直流母线传递有功功率
            I_ref_mag = -pid_out;                                           // 转换为当前PLL相位方向所需的负幅值约定
        }
    }

    if((rectifier_enable != 0) && (rectifier_fault == 0) &&                 // 主动整流命令与局部保护均允许输出
       (system_fault == 0) && (rectifier_pwm_start_stage != 0))             // 已进入整流PWM预装或正常运行阶段
    {
        i_l_ref = I_ref_mag * cosf(theta);                                  // 生成与输入电压同相的瞬时电流参考
        pll_out = i_l_ref;                                                  // 同步旧变量供在线观察瞬时电流参考
        pid2.ref = i_l_ref;                                                 // 逐拍更新输入电流比例控制器参考值
        u_i_out = PID2_Cal(I_in);                                           // 通过PID2统一计算比例电流补偿电压
        err_i = pid2.Err;                                                   // 同步PID2误差供在线观察
        if(u_i_out > CURRENT_CTRL_VOLTAGE_LIMIT)                             // 限制正向电流补偿电压
            u_i_out = CURRENT_CTRL_VOLTAGE_LIMIT;                           // 防止大电流误差直接把调制量推入饱和
        else if(u_i_out < -CURRENT_CTRL_VOLTAGE_LIMIT)                       // 限制反向电流补偿电压
            u_i_out = -CURRENT_CTRL_VOLTAGE_LIMIT;                          // 保持电流补偿电压上下限对称
        i_ctrl = u_i_out + RECTIFIER_FEEDFORWARD_GAIN * U_in;               // 叠加0.5倍输入电压前馈
        middle = i_ctrl;                                                    // 同步旧中间变量便于在线观察
        if(U_bus > BUS_CONTROL_MIN_VOLTAGE) v_dc_bus = U_bus;               // 正常时使用实时母线电压完成调制解耦
        else v_dc_bus = BUS_CONTROL_MIN_VOLTAGE;                            // 母线过低时使用5V下限避免除零
        Di = i_ctrl / v_dc_bus;                                             // 把桥侧电压指令归一化为单极性调制量
        if(Di > RECTIFIER_MODULATION_LIMIT) Di = RECTIFIER_MODULATION_LIMIT; // 保留原正向调制限幅
        else if(Di < -RECTIFIER_MODULATION_LIMIT) Di = -RECTIFIER_MODULATION_LIMIT; // 保留原负向调制限幅
        D1 = 0.5f * (1.0f + Di);                                            // ePWM4左桥臂占空比采用(1+Di)/2
        EPwm4Regs.CMPA.bit.CMPA = (Uint16)(EPWM_TIMER_TBPRD * D1);          // 更新整流左桥臂中心对齐比较值
        EPwm5Regs.CMPA.bit.CMPA = (Uint16)(EPWM_TIMER_TBPRD * 0.5f * (1.0f - Di)); // 更新整流右桥臂中心对齐比较值
        PWM_ReleaseRectifier();                                             // 独立完成ePWM4/5预装等待和Trip释放
    }
    else
    {
        PWM_TripRectifier();                                                // 非主动整流状态始终保持ePWM4/5 Trip关断
        pll_out = 0.0f;                                                     // 关断主动整流后清零瞬时电流参考观察量
        D1 = 0.5f;                                                          // 显示占空比恢复到中点
        middle = 0.0f;                                                      // 清零整流电流环中间量
        i_l_ref = 0.0f;                                                     // 清零整流瞬时电流参考
        err_i = 0.0f;                                                       // 清零整流比例电流误差
        u_i_out = 0.0f;                                                     // 清零整流比例控制输出
        i_ctrl = 0.0f;                                                      // 清零整流桥侧电压指令
        Di = 0.0f;                                                          // 清零整流差模调制量
        pid2.ref = 0.0f;                                                    // 清零停机状态下的PID2参考
        pid2.Xin = 0.0f;                                                    // 清零停机状态下的PID2输入
        pid2.Err = 0.0f;                                                    // 清零停机状态下的PID2误差
        pid2.Integral = 0.0f;                                               // 清零停机状态下的PID2积分
        pid2.result = 0.0f;                                                 // 清零停机状态下的PID2输出
    }

    if((inverter_pwm_start_stage != 0) && (system_fault == 0))              // 逆变处于预装、待释放或正常运行状态
    {
        theta_a += 2.0f * pi * (float)output_freq_hz * Ts;                  // 每个20kHz控制周期按当前45～505Hz命令推进输出相位
        if(theta_a >= 2.0f * pi) theta_a -= 2.0f * pi;                      // A相相位超过一周时回绕
        theta_b = theta_a + 2.0944f;                                        // B相保持超前A相120度
        if(theta_b >= 2.0f * pi) theta_b -= 2.0f * pi;                      // B相相位超过一周时回绕
        theta_c = theta_a - 2.0944f;                                        // C相保持滞后A相120度
        if(theta_c < 0.0f) theta_c += 2.0f * pi;                            // C相负角度回绕至0～2π

        bus_feedforward_gain = U_BUS_REF / U_bus_ff;                        // 母线升高时主动减小调制度，降低时主动增大调制度
        if(bus_feedforward_gain > BUS_FEEDFORWARD_GAIN_MAX)                 // 限制欠压或采样扰动造成的前馈突增
            bus_feedforward_gain = BUS_FEEDFORWARD_GAIN_MAX;
        else if(bus_feedforward_gain < BUS_FEEDFORWARD_GAIN_MIN)            // 限制母线过高时调制度突降
            bus_feedforward_gain = BUS_FEEDFORWARD_GAIN_MIN;
        inverter_modulation = (inverter_soft_gain / M) * bus_feedforward_gain;// 总电压环、软启动和快速母线前馈串联
        if(inverter_modulation > INVERTER_MODULATION_LIMIT)                 // 保证叠加1%分相补偿后仍处于SVPWM线性区
            inverter_modulation = INVERTER_MODULATION_LIMIT;
        else if(inverter_modulation < 0.0f)
            inverter_modulation = 0.0f;
        modulation_a = inverter_modulation * (1.0f + phase_trim_a);         // A相叠加独立幅值补偿
        modulation_b = inverter_modulation * (1.0f + phase_trim_b);         // B相叠加独立幅值补偿
        modulation_c = inverter_modulation * (1.0f + phase_trim_c);         // C相叠加独立幅值补偿
        // 按三相独立调制度注入同一个SVPWM零序共模分量
        SVPWM_Calculate(theta_a, theta_b, theta_c, modulation_a,
                        modulation_b, modulation_c, &Da, &Db, &Dc);
        EPwm1Regs.CMPA.bit.CMPA = (Uint16)(EPWM_TIMER_TBPRD * Da);           // 将SVPWM A相占空比转换为中心对齐比较值
        EPwm2Regs.CMPA.bit.CMPA = (Uint16)(EPWM_TIMER_TBPRD * Db);           // 将SVPWM B相占空比转换为中心对齐比较值
        EPwm3Regs.CMPA.bit.CMPA = (Uint16)(EPWM_TIMER_TBPRD * Dc);           // 将SVPWM C相占空比转换为中心对齐比较值

        N_c2++;                                                             // 保留10ms慢速控制与软启动调度节拍
        if(N_c2 >= N)                                                       // 每200个20kHz采样点更新一次慢速控制状态
        {
            N_c2 = 0;                                                       // 清零逆变统计窗口计数
            U_out_rms = sqrtf(output_common_sq_f);                          // 公共电压环使用快速三相平方IIR的有效值
            U_oa_rms = sqrtf(output_phase_a_sq_f);                          // 分相补偿使用慢速A相有效值
            U_ob_rms = sqrtf(output_phase_b_sq_f);                          // 分相补偿使用慢速B相有效值
            U_oc_rms = sqrtf(output_phase_c_sq_f);                          // 分相补偿使用慢速C相有效值
            U_phase_avg = (U_oa_rms + U_ob_rms + U_oc_rms) * 0.3333333f;    // 保存三相平均值供在线观察
            if(inverter_soft_stop_active != 0)                              // KEY6关闭请求逆变软关断
            {
                inverter_soft_gain -= INVERTER_SOFT_GAIN_DOWN_STEP;         // 每10ms降低0.05输出幅值系数
                if(inverter_soft_gain < 0.0f) inverter_soft_gain = 0.0f;    // 防止软关断系数变为负数
            }
            else if(inverter_enable != 0)                                   // KEY6仍保持逆变运行请求
            {
                inverter_soft_gain += INVERTER_SOFT_GAIN_UP_STEP;           // 每10ms增加0.02输出幅值系数
                if(inverter_soft_gain > 1.0f) inverter_soft_gain = 1.0f;    // 正常运行幅值系数最大为1
            }

            pida.ref = U_OUT_REF * inverter_soft_gain;                      // 电压环目标与软启停幅值同步变化以避免相互对抗
            M += PIDa_Cal(U_out_rms);                                       // 保留原逆变电压环对调制分母M的增量调节
            if(M >= 8) M = 8;                                               // 保留原调制分母上限
            if(M <= 1.8f) M = 1.8f;                                         // SVPWM下限保留约1.9%最小占空比裕量

            if((inverter_pwm_start_stage == 3) &&                           // 仅在PWM已正常释放且软启动基本完成时启用平衡环
               (inverter_soft_gain >= PHASE_BALANCE_ENABLE_GAIN) &&
               (inverter_soft_stop_active == 0))
            {
                if(phase_balance_settle_windows < PHASE_BALANCE_SETTLE_WINDOWS)
                    phase_balance_settle_windows++;
                if(phase_balance_settle_windows >= PHASE_BALANCE_SETTLE_WINDOWS)
                {
                    PhaseBalance_Calculate(U_oa_rms, U_ob_rms, U_oc_rms,    // 计算下一控制周期使用的零和三相补偿
                                           &phase_trim_a, &phase_trim_b,
                                           &phase_trim_c);
                }
                else
                {
                    phase_trim_a = 0.0f;
                    phase_trim_b = 0.0f;
                    phase_trim_c = 0.0f;
                }
            }
            else
            {
                phase_balance_settle_windows = 0U;                          // 软启动或软关断期间禁用独立补偿
                phase_trim_a = 0.0f;
                phase_trim_b = 0.0f;
                phase_trim_c = 0.0f;
            }

            if((inverter_soft_stop_active != 0) &&                          // 软关断过程已经把幅值降至零
               (inverter_soft_gain <= 0.0f))
            {
                PWM_TripInverter();                                         // 零幅值后再Trip ePWM1～3完成软关断
                inverter_pwm_start_stage = 0;                               // 下次运行必须重新完成CMPA预装
                inverter_soft_stop_active = 0;                              // 结束本次软关断状态
            }
        }

        if(inverter_pwm_start_stage != 0) PWM_ReleaseInverter();            // 独立完成ePWM1～3预装等待和Trip释放
    }
    else                                                                    // 逆变没有运行或已经被保护关断
    {
        PWM_TripInverter();                                                 // 保持ePWM1～3的One-Shot Trip锁存
        EPwm1Regs.CMPA.bit.CMPA = 0;                                        // 清零逆变A相比较值影子寄存器
        EPwm2Regs.CMPA.bit.CMPA = 0;                                        // 清零逆变B相比较值影子寄存器
        EPwm3Regs.CMPA.bit.CMPA = 0;                                        // 清零逆变C相比较值影子寄存器
        N_c2 = 0;                                                           // 清零逆变输出统计窗口计数
        U_out_rms = 0.0f;                                                   // 关断状态下清零逆变窗口输出值
        U_oa_rms = 0.0f;                                                    // 清零三相慢速有效值，防止下次启动沿用旧状态
        U_ob_rms = 0.0f;
        U_oc_rms = 0.0f;
        U_phase_avg = 0.0f;
        output_common_sq_f = 0.0f;                                          // 清零公共及分相IIR状态
        output_phase_a_sq_f = 0.0f;
        output_phase_b_sq_f = 0.0f;
        output_phase_c_sq_f = 0.0f;
        phase_balance_settle_windows = 0U;                                  // 下次满幅后重新等待500ms
        phase_trim_a = 0.0f;                                                // 关断时禁止残留相间补偿
        phase_trim_b = 0.0f;
        phase_trim_c = 0.0f;
        inverter_modulation = 0.0f;
        modulation_a = 0.0f;
        modulation_b = 0.0f;
        modulation_c = 0.0f;
        theta_a = 0.0f;                                                     // 把A相输出相位复位到零点
        theta_b = 2.0944f;                                                  // 把B相输出相位复位到超前120度
        theta_c = 4.1888f;                                                  // 把C相输出相位复位到滞后120度的等效正角
    }

    N_freq_key++;                                                           // 独立累加KEY1/KEY2频率调节分频计数
    if(N_freq_key >= FREQ_KEY_REPEAT_ISR_DIV)                               // 每5000个20kHz控制周期即250ms处理一次频率按键
    {
        N_freq_key = 0;                                                     // 清零频率按键重复分频计数
        if(KEY_H1 == 0) KEY_Control(KEY1_PRESS);                            // KEY1按下或长按时把频率增加1Hz
        else if(KEY_H2 == 0) KEY_Control(KEY2_PRESS);                       // KEY2按下或长按时把频率减小1Hz
    }

    N_key++;                                                                // 独立累加KEY3～KEY6通用按键扫描分频计数
    if(N_key >= GENERAL_KEY_SCAN_ISR_DIV)                                   // 保持原每10000个控制周期即500ms扫描一次
    {
        N_key = 0;                                                          // 清零按键扫描分频计数
        key = KEY_Scan(0);                                                  // 扫描目标板KEY3～KEY6
        KEY_Control(key);                                                   // 执行按键对应的模式或设定值操作
    }
    tag = ((rectifier_enable != 0) ? 2 : 0) + ((inverter_enable != 0) ? 1 : 0); // 实时组合独立模式状态供在线观察

    N_oled++;                                                               // 累加OLED刷新分频计数
    if(N_oled >= OLED_REFRESH_DIVIDER)                                      // 每1000个控制周期即50ms请求刷新一行
    {
        N_oled = 0;                                                         // 清零OLED刷新分频计数器
        oled_refresh_request = 1;                                           // 通知主循环执行一次单行OLED刷新
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                                  // 清除ADCA INT1标志以允许下一次中断
    if(AdcaRegs.ADCINTOVF.bit.ADCINT1 == 1)                                 // 检查控制ISR是否发生ADC中断溢出
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;                              // 清除ADCA INT1溢出标志
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                              // 清除ADCA INT1标志以允许下一次中断
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;                                 // 应答PIE第1组以接收下一次ADC中断
}

//******************* 锁相环 *******************//
void PLL1(float UI)                                                         // SOGI-PLL输入电压锁相计算
{
    x = UI - Ua;                                                            // 计算输入电压与SOGI α轴输出之差
    Ua = Ua + (x * k - Ub) * W0 * Ts;                                       // 按已核对差分方程更新SOGI α轴分量
    Ub = Ub + Ua * W0 * Ts;                                                 // 按已核对差分方程更新SOGI β轴分量
    Ud = cosf(theta) * Ua + sinf(theta) * Ub;                               // 将αβ分量变换到同步旋转d轴
    Uq = cosf(theta) * Ub - sinf(theta) * Ua;                               // 将αβ分量变换到同步旋转q轴
    err = 0.0f - Uq;                                                        // 以Uq趋近零作为PLL锁相目标
    result = kp * err + ki * (err - last_err);                              // 按已核对写法计算本拍角频率修正量
    last_err = err;                                                         // 保存本拍误差供下一拍差分使用
    W1 -= result;                                                           // 累加修正PLL估算角频率
    if(W1 >= 115.0f * pi) W1 = 115.0f * pi;                                // 将PLL最高频率限制为57.5Hz
    if(W1 <= 85.0f * pi) W1 = 85.0f * pi;                                  // 将PLL最低频率限制为42.5Hz
    W1_pi = W1 / (2.0f * pi);                                               // 把估算角频率换算成Hz供在线观察
    theta += W1 * Ts;                                                       // 使用当前估算角频率推进PLL相位
    if(theta >= 2.0f * pi) theta -= 2.0f * pi;                              // 相位超过一周时回绕到0～2π
    else if(theta <= 0.0f) theta += 2.0f * pi;                              // 相位为负或零时按已核对写法执行回卷
}

//******************* PID1：母线电压外环 *******************//
void PID1_Init(void)                                                        // 初始化母线电压外环参数
{
    pid1.Xin = 0;                                                           // 初始化或更新控制器测量输入
    pid1.ref = U_BUS_REF;                                                   // 初始化母线电压外环参考值
    pid1.Err = 0;                                                           // 计算或清零当前控制误差
    pid1.Err_last = 0;                                                      // 保存或清零上一拍误差
    pid1.Kp = 0.05f;                                                        // 仿照单极性程序设置母线电压外环比例系数
    pid1.Ki = 0.001f;                                                       // 仿照单极性程序设置母线电压外环积分系数
    pid1.result = 0;                                                        // 计算或清零控制器当前输出
}

float PID1_Cal(float u)                                                     // 计算母线电压外环增量
{
    pid1.Xin = u;                                                           // 初始化或更新控制器测量输入
    pid1.Err = pid1.ref - pid1.Xin;                                         // 母线欠压时产生正误差并提高输入电流幅值
    pid1.result = pid1.Kp * (pid1.Err - pid1.Err_last) + pid1.Ki * pid1.Err; // 计算或清零控制器当前输出
    pid1.Err_last = pid1.Err;                                               // 保存或清零上一拍误差
    return pid1.result;                                                     // 返回母线电压外环增量
}

//******************* PID2：输入电流PI控制器 *******************//
void PID2_Init(void)                                                        // 初始化输入电流PI控制参数
{
    pid2.Xin = 0;                                                           // 初始化或更新控制器测量输入
    pid2.ref = 0;                                                           // 初始化控制器参考值
    pid2.Err = 0;                                                           // 计算或清零当前控制误差
    pid2.Kp = RECTIFIER_CURRENT_KP;                                         // 使用整流输入电流比例增益
    pid2.Ki = RECTIFIER_CURRENT_KI;                                         // 使用保守积分增益缓慢消除稳态误差
    pid2.Integral = 0;                                                      // 清零输入电流积分状态
    pid2.result = 0;                                                        // 计算或清零控制器当前输出
}

float PID2_Cal(float u)                                                     // 计算输入电流PI控制输出
{
    pid2.Xin = u;                                                           // 初始化或更新控制器测量输入
    pid2.Err = pid2.ref - pid2.Xin;                                         // 计算或清零当前控制误差
    pid2.Integral += pid2.Err;                                              // 累加输入电流误差
    if(pid2.Integral > RECTIFIER_CURRENT_INTEGRAL_LIMIT)                    // 限制正向积分状态
        pid2.Integral = RECTIFIER_CURRENT_INTEGRAL_LIMIT;                   // 最大积分补偿为0.3V
    else if(pid2.Integral < -RECTIFIER_CURRENT_INTEGRAL_LIMIT)              // 限制反向积分状态
        pid2.Integral = -RECTIFIER_CURRENT_INTEGRAL_LIMIT;                  // 最小积分补偿为-0.3V
    pid2.result = pid2.Kp * pid2.Err + pid2.Ki * pid2.Integral;             // 合成比例与积分电流补偿
    return pid2.result;                                                     // 返回输入电流PI控制输出
}

static void SVPWM_Calculate(float phase_a, float phase_b, float phase_c,     // 使用最大最小值共模注入法计算线性区SVPWM
                            float modulation_a, float modulation_b,
                            float modulation_c, float *duty_a,
                            float *duty_b, float *duty_c)
{
    float ref_a = modulation_a * sinf(phase_a);                             // 生成叠加A相平衡补偿后的基波参考
    float ref_b = modulation_b * sinf(phase_b);                             // 生成叠加B相平衡补偿后的基波参考
    float ref_c = modulation_c * sinf(phase_c);                             // 生成叠加C相平衡补偿后的基波参考
    float sv_max = ref_a;                                                    // 初始化三相参考最大值
    float sv_min = ref_a;                                                    // 初始化三相参考最小值
    float sv_offset;                                                         // 保存SVPWM零序共模偏置

    if(ref_b > sv_max) sv_max = ref_b;                                      // 更新三相参考最大值
    if(ref_c > sv_max) sv_max = ref_c;                                      // 更新三相参考最大值
    if(ref_b < sv_min) sv_min = ref_b;                                      // 更新三相参考最小值
    if(ref_c < sv_min) sv_min = ref_c;                                      // 更新三相参考最小值

    sv_offset = -0.5f * (sv_max + sv_min);                                  // 居中有效矢量并等分零矢量作用时间
    *duty_a = 0.5f + ref_a + sv_offset;                                     // 叠加共模偏置得到A相占空比
    *duty_b = 0.5f + ref_b + sv_offset;                                     // 叠加共模偏置得到B相占空比
    *duty_c = 0.5f + ref_c + sv_offset;                                     // 叠加共模偏置得到C相占空比

    if(*duty_a > 1.0f) *duty_a = 1.0f;                                     // 防止异常调制度导致A相比较值越界
    else if(*duty_a < 0.0f) *duty_a = 0.0f;                                // 防止异常调制度导致A相比较值越界
    if(*duty_b > 1.0f) *duty_b = 1.0f;                                     // 防止异常调制度导致B相比较值越界
    else if(*duty_b < 0.0f) *duty_b = 0.0f;                                // 防止异常调制度导致B相比较值越界
    if(*duty_c > 1.0f) *duty_c = 1.0f;                                     // 防止异常调制度导致C相比较值越界
    else if(*duty_c < 0.0f) *duty_c = 0.0f;                                // 防止异常调制度导致C相比较值越界
}

static void PhaseBalance_Calculate(float ua_rms, float ub_rms, float uc_rms,// 计算不改变公共调制度的三相独立幅值补偿
                                   float *trim_a, float *trim_b,
                                   float *trim_c)
{
    float phase_average = (ua_rms + ub_rms + uc_rms) * 0.3333333f;          // 以三相平均有效值作为平衡目标
    float trim_mean;                                                         // 保存三相补偿均值以强制零和
    float abs_a, abs_b, abs_c;                                              // 保存补偿绝对值
    float max_abs;                                                           // 保存三相最大补偿绝对值
    float trim_scale;                                                        // 同比例缩放因子，避免独立削顶破坏零和

    *trim_a = PHASE_BALANCE_KP * (phase_average - ua_rms);                  // 电压偏低的相增加调制度
    *trim_b = PHASE_BALANCE_KP * (phase_average - ub_rms);
    *trim_c = PHASE_BALANCE_KP * (phase_average - uc_rms);
    trim_mean = (*trim_a + *trim_b + *trim_c) * 0.3333333f;                 // 去除浮点计算及采样造成的公共分量
    *trim_a -= trim_mean;
    *trim_b -= trim_mean;
    *trim_c -= trim_mean;

    abs_a = (*trim_a >= 0.0f) ? *trim_a : -*trim_a;                         // 不依赖库函数计算绝对值，缩短ISR路径
    abs_b = (*trim_b >= 0.0f) ? *trim_b : -*trim_b;
    abs_c = (*trim_c >= 0.0f) ? *trim_c : -*trim_c;
    max_abs = abs_a;
    if(abs_b > max_abs) max_abs = abs_b;
    if(abs_c > max_abs) max_abs = abs_c;
    if(max_abs > PHASE_BALANCE_TRIM_LIMIT)                                  // 任一相超过1%时三相同比例收缩
    {
        trim_scale = PHASE_BALANCE_TRIM_LIMIT / max_abs;
        *trim_a *= trim_scale;
        *trim_b *= trim_scale;
        *trim_c *= trim_scale;
    }
}

//******************* PIDa：三相输出电压环 *******************//
void PIDa_Init(void)                                                        // 初始化逆变输出电压控制参数
{
    pida.Xin = 0;                                                           // 初始化或更新控制器测量输入
    pida.ref = 0;                                                           // 初始化控制器参考值
    pida.Err = 0;                                                           // 计算或清零当前控制误差
    pida.Err_last = 0;                                                      // 保存或清零上一拍误差
    pida.Kp = 0.025;                                                         // 设置比例系数
    pida.Ki = 0.010;                                                        // 设置积分系数
    pida.result = 0;                                                        // 计算或清零控制器当前输出
}

float PIDa_Cal(float u)                                                     // 计算逆变调制分母增量
{
    pida.Xin = u;                                                           // 初始化或更新控制器测量输入
    pida.Err = pida.Xin - pida.ref;                                         // 计算或清零当前控制误差
    pida.result = pida.Kp * (pida.Err - pida.Err_last) + pida.Ki * pida.Err; // 计算或清零控制器当前输出
    pida.Err_last = pida.Err;                                               // 保存或清零上一拍误差
    if(fabsf(pida.Err) > 0.01f) return pida.result;                         // 输出误差超出死区时返回调节增量
    return 0;                                                               // 误差位于死区时返回零增量
}

//******************* OLED显示 *******************//
void OLED_output(void)                                                      // 刷新设定值和测量值显示
{
    float oled_bus_value;                                                   // 保存本次刷新使用的实时母线电压快照
    float oled_out_value;                                                   // 保存本次刷新使用的三相输出电压快照
    float oled_di_value;                                                    // 保存本次刷新使用的整流调制量快照
    DINT;                                                                   // 短暂屏蔽ISR，保证三个32位浮点快照内部一致
    oled_bus_value = U_bus;                                                 // 获取实时母线电压供全部模式统一显示
    oled_out_value = U_av;                                                  // 获取实时三相输出等效相电压供全部模式统一显示
    oled_di_value = Di;                                                     // 只读取一次Di，避免符号和幅值来自不同控制周期
    EINT;                                                                   // 快照完成后立即恢复控制中断
    if(oled_bus_value < 0.0f) oled_bus_value = 0.0f;                        // 钳除零点偏差产生的负母线显示值
    if(oled_out_value < 0.0f) oled_out_value = 0.0f;                        // 防止异常负数进入不支持负数的OLED浮点函数
    if(oled_bus_value > 99.0f) oled_bus_value = 99.0f;                      // 限制显示范围以避免超过固定字符区域
    if(oled_out_value > 99.0f) oled_out_value = 99.0f;                      // 限制输出显示范围以保持四行布局稳定

    switch(oled_row)                                                        // 每次调用只刷新一行以降低后台OLED通信占用
    {
        case 0:                                                             // 第0行分别显示整流命令、逆变命令、故障和调制分母
            OLED_ShowNum(1, 0, rectifier_enable, 1);                        // R后显示主动整流命令0或1
            OLED_ShowNum(4, 0, inverter_enable, 1);                         // I后显示逆变运行命令0或1
            OLED_ShowNum(7, 0, flag, 1);                                    // F后显示当前故障码0、1或2
            OLED_ShowFloat(10, 0, M, 3);                                    // M后显示逆变当前调制分母
            break;                                                          // 本次单行刷新结束

        case 1:                                                             // 第1行显示母线参考和母线实测值
            OLED_ShowFloat(3, 1, U_BUS_REF, 3);                             // BR后显示母线电压参考值
            OLED_ShowFloat(10, 1, oled_bus_value, 3);                       // B后显示当前母线实时采样值
            break;                                                          // 本次单行刷新结束

        case 2:                                                             // 第2行显示输出参考和输出实测值
            OLED_ShowFloat(3, 2, U_OUT_REF, 3);                             // OR后显示三相输出相电压参考值
            OLED_ShowFloat(10, 2, oled_out_value, 3);                       // O后显示当前三相输出等效相电压
            break;                                                          // 本次单行刷新结束

        default:                                                            // 第3行显示逆变输出频率命令和带符号的整流调制量
            OLED_ShowNum(1, 3, output_freq_hz, 4);                          // F后以整数显示45～505Hz频率命令
            OLED_ShowChar(10, 3, (oled_di_value < 0.0f) ? '-' : '+');       // DI后显式显示整流差模调制量快照符号
            OLED_ShowFloat(11, 3, fabsf(oled_di_value), 3);                 // 符号后显示同一快照的0～0.95幅值
            break;                                                          // 本次单行刷新结束
    }

    oled_row++;                                                             // 下一次刷新下一行
    if(oled_row >= 4) oled_row = 0;                                         // 第3行完成后回到第0行循环刷新
}

//******************* 按键控制 *******************//
void KEY_Control(int key_value)                                             // 根据按键修改模式或参考值
{
    switch(key_value)                                                       // 根据按键码选择对应操作
    {
        case KEY1_PRESS:                                                    // KEY1固定增加三相逆变输出频率
            if(output_freq_hz < OUTPUT_FREQ_MAX_HZ)                         // 已到505Hz上限时保持不变
                output_freq_hz += OUTPUT_FREQ_STEP_HZ;                      // 每次扫描事件把输出频率增加1Hz
            break;                                                          // 结束当前按键分支

        case KEY2_PRESS:                                                    // KEY2固定减小三相逆变输出频率
            if(output_freq_hz > OUTPUT_FREQ_MIN_HZ)                         // 已到45Hz下限时保持不变
                output_freq_hz -= OUTPUT_FREQ_STEP_HZ;                      // 每次扫描事件把输出频率减小1Hz
            break;                                                          // 结束当前按键分支

        case KEY3_PRESS:                                                    // KEY3切换主动整流，并联锁保证逆变先启动
            if(rectifier_enable == 0)                                       // 当前处于被动整流或整流Trip状态
            {
                if(U_bus < BUS_OVERVOLTAGE_LIMIT)                           // 母线已经退出过压区才允许重新确认启动
                {
                    system_fault = 0;                                       // 同时清除可能存在的预留全局故障锁存
                    rectifier_fault = 0;                                    // 清除整流局部故障锁存
                    inverter_enable = 1;                                    // KEY3同时保证先提出逆变运行请求
                    rectifier_enable = 1;                                   // 请求整流等待逆变PWM正常后再启动ePWM4/5
                    flag = 0;                                               // 清除旧故障显示
                }
            }
            else                                                            // 当前主动整流已经处于开启请求
            {
                rectifier_enable = 0;                                       // 撤销主动整流命令但不改变逆变命令
                rectifier_pwm_start_stage = 0;                              // 下次KEY3开启时重新执行CMPA预装
                PWM_TripRectifier();                                        // 本次按键处理当拍立即Trip ePWM4/5
            }
            break;                                                          // 结束当前按键分支

        case KEY4_PRESS:                                                    // KEY4固定增加逆变侧输出相电压目标值
            U_OUT_REF += 0.5f;                                              // 每次扫描事件把逆变输出目标增加0.5V
            break;                                                          // 结束当前按键分支

        case KEY5_PRESS:                                                    // KEY5固定减小逆变侧输出相电压目标值
            U_OUT_REF -= 0.5f;                                              // 每次扫描事件把逆变输出目标减小0.5V
            break;                                                          // 结束当前按键分支

        case KEY6_PRESS:                                                    // KEY6切换三相逆变，关断时先停止主动整流
            if(inverter_enable == 0)                                        // 当前逆变侧没有运行请求
            {
                if(U_bus < BUS_OVERVOLTAGE_LIMIT)                           // 母线已经退出过压区才接受新的逆变运行请求
                {
                    system_fault = 0;                                       // 清除可能存在的预留全局故障锁存
                    inverter_enable = 1;                                    // 请求逆变立即进入PWM预装和软启动流程
                }
            }
            else                                                            // 当前逆变侧已有运行请求
            {
                if(rectifier_enable != 0)                                   // 主动整流仍有运行请求时必须先停止整流
                {
                    rectifier_enable = 0;                                   // 撤销主动整流命令
                    rectifier_pwm_start_stage = 0;                          // 清零整流PWM启动阶段
                    PWM_TripRectifier();                                    // 立即Trip ePWM4/5，确保整流先于逆变关断
                }
                inverter_enable = 0;                                        // 撤销逆变运行命令并由控制ISR执行软关断
            }
            break;                                                          // 结束当前按键分支

        case KEY_UNPRESS:                                                   // 无按键时不执行操作
        default:                                                            // 未定义按键码同样不执行操作
            break;                                                          // 结束当前按键分支
    }
    tag = ((rectifier_enable != 0) ? 2 : 0) + ((inverter_enable != 0) ? 1 : 0); // 组合状态仅供在线观察，不参与任一控制条件
}

char KEY_Scan(char key_mode)                                                // 仿照07192140工程读取六个低电平有效按键
{
    static int key3_latched = 0;                                            // KEY3按下后锁存一次，必须检测到松开才允许再次切换模式
    static int key6_latched = 0;                                            // KEY6按下后锁存一次，防止长按反复启停逆变
    if(KEY_H3 != 0) key3_latched = 0;                                       // KEY3松开后解除锁存，重新允许下一次按下事件
    if(KEY_H6 != 0) key6_latched = 0;                                       // KEY6松开后解除锁存，重新允许下一次按下事件

    if(key_mode == 0)                                                       // 当前调用使用模式0时依次检测六个按键
    {
        if((KEY_H3 == 0) && (key3_latched == 0))                            // KEY3本次按下尚未产生过模式切换事件
        {
            key3_latched = 1;                                               // 锁存本次按下，防止长按反复启停主动整流
            return KEY3_PRESS;                                              // 每次完整按下与松开过程只返回一次KEY3事件
        }
        if(KEY_H4 == 0) return KEY4_PRESS;                                  // 长按KEY4时保持每500ms重复一次升压事件
        if(KEY_H5 == 0) return KEY5_PRESS;                                  // 长按KEY5时保持每500ms重复一次降压事件
        if((KEY_H6 == 0) && (key6_latched == 0))                            // KEY6本次按下尚未产生过逆变模式切换
        {
            key6_latched = 1;                                               // 锁存本次KEY6按下直到检测到松开
            return KEY6_PRESS;                                              // 每次完整按下与松开过程只返回一次KEY6事件
        }
    }
    return KEY_UNPRESS;                                                     // 未检测到按键时返回未按下状态
}

void Init_KEY(void)                                                         // 配置目标板六个按键输入
{
    EALLOW;                                                                 // 允许访问受保护寄存器

    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);                               // 配置对应按键GPIO为标准数字输入焊盘
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_IN);                            // 配置对应按键GPIO为输入方向
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;                                     // 使能对应按键GPIO内部上拉

    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);                               // 配置对应按键GPIO为标准数字输入焊盘
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);                            // 配置对应按键GPIO为输入方向
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;                                     // 使能对应按键GPIO内部上拉

    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);                               // 配置对应按键GPIO为标准数字输入焊盘
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);                            // 配置对应按键GPIO为输入方向
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;                                     // 使能对应按键GPIO内部上拉

    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);                               // 配置对应按键GPIO为标准数字输入焊盘
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);                            // 配置对应按键GPIO为输入方向
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;                                     // 使能对应按键GPIO内部上拉

    GPIO_setAnalogMode(16, GPIO_ANALOG_DISABLED);                           // 参考07192140工程关闭GPIO16模拟功能供KEY5使用
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);                               // 配置KEY5对应GPIO16为标准数字输入焊盘
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);                            // 配置KEY5对应GPIO16为输入方向
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;                                     // 使能KEY5对应GPIO16内部上拉

    GPIO_setAnalogMode(39, GPIO_ANALOG_DISABLED);                           // 参考07192140工程关闭GPIO39模拟功能供KEY6使用
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);                               // 配置KEY6对应GPIO39为标准数字输入焊盘
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);                            // 配置KEY6对应GPIO39为输入方向
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;                                     // 使能KEY6对应GPIO39内部上拉

    EDIS;                                                                   // 重新禁止访问受保护寄存器
}

//******************* PWM安全控制辅助函数 *******************//
static void PWM_ForceAllLow(void)                                           // 将六组ePWM的A/B输出强制拉低
{
    EALLOW;                                                                 // 允许访问受保护的Trip Zone软件触发寄存器
    EPwm1Regs.TZFRC.bit.OST = 1;                                            // 从死区模块之后立即锁存关断逆变A相A/B输出
    EPwm2Regs.TZFRC.bit.OST = 1;                                            // 从死区模块之后立即锁存关断逆变B相A/B输出
    EPwm3Regs.TZFRC.bit.OST = 1;                                            // 从死区模块之后立即锁存关断逆变C相A/B输出
    EPwm4Regs.TZFRC.bit.OST = 1;                                            // 从死区模块之后立即锁存关断整流左桥臂A/B输出
    EPwm5Regs.TZFRC.bit.OST = 1;                                            // 从死区模块之后立即锁存关断整流右桥臂A/B输出
    EPwm6Regs.TZFRC.bit.OST = 1;                                            // 锁存关断未使用的ePWM6 A/B输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
}

static void PWM_TripInverter(void)                                          // 只关断逆变侧ePWM1～3并保持整流侧当前状态
{
    EALLOW;                                                                 // 允许访问逆变侧Trip Zone软件触发寄存器
    EPwm1Regs.TZFRC.bit.OST = 1;                                            // 立即锁存关断逆变A相A/B输出
    EPwm2Regs.TZFRC.bit.OST = 1;                                            // 立即锁存关断逆变B相A/B输出
    EPwm3Regs.TZFRC.bit.OST = 1;                                            // 立即锁存关断逆变C相A/B输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
}

static void PWM_TripRectifier(void)                                         // 只关断整流侧ePWM4/5并保持逆变侧当前状态
{
    EALLOW;                                                                 // 允许访问整流侧Trip Zone软件触发寄存器
    EPwm4Regs.TZFRC.bit.OST = 1;                                            // 立即锁存关断整流左桥臂A/B输出
    EPwm5Regs.TZFRC.bit.OST = 1;                                            // 立即锁存关断整流右桥臂A/B输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
}

static void PWM_ReleaseInverter(void)                                       // 按逆变独立启动阶段释放ePWM1～3
{
    EALLOW;                                                                 // 允许访问逆变侧Trip Zone清除与软件触发寄存器
    if(inverter_pwm_start_stage == 1)                                       // 第一拍仅把三相计算值写入CMPA影子寄存器
    {
        inverter_pwm_start_stage = 2;                                       // 等待下一次TBCTR归零装载新的三相CMPA
    }
    else if(inverter_pwm_start_stage == 2)                                  // 第二拍ISR到来时CMPA活动值已经完成装载
    {
        EPwm1Regs.TZCLR.bit.OST = 1;                                        // 解除逆变A相One-Shot Trip关断锁存
        EPwm2Regs.TZCLR.bit.OST = 1;                                        // 解除逆变B相One-Shot Trip关断锁存
        EPwm3Regs.TZCLR.bit.OST = 1;                                        // 解除逆变C相One-Shot Trip关断锁存
        inverter_pwm_start_stage = 3;                                       // 标记三相逆变PWM已经进入正常运行阶段
    }
    else if(inverter_pwm_start_stage != 3)                                  // 非法阶段值下不允许逆变桥输出
    {
        EPwm1Regs.TZFRC.bit.OST = 1;                                        // 异常状态保持逆变A相关断
        EPwm2Regs.TZFRC.bit.OST = 1;                                        // 异常状态保持逆变B相关断
        EPwm3Regs.TZFRC.bit.OST = 1;                                        // 异常状态保持逆变C相关断
    }
    EDIS;                                                                   // 重新禁止访问受保护寄存器
}

static void PWM_ReleaseRectifier(void)                                      // 按整流独立启动阶段释放ePWM4/5
{
    EALLOW;                                                                 // 允许访问整流侧Trip Zone清除与软件触发寄存器
    if(rectifier_pwm_start_stage == 1)                                      // 第一拍仅把整流计算值写入CMPA影子寄存器
    {
        rectifier_pwm_start_stage = 2;                                      // 等待下一次TBCTR归零装载ePWM4/5比较值
    }
    else if(rectifier_pwm_start_stage == 2)                                 // 第二拍ISR到来时整流CMPA已经完成装载
    {
        EPwm4Regs.TZCLR.bit.OST = 1;                                        // 解除整流左桥臂One-Shot Trip关断锁存
        EPwm5Regs.TZCLR.bit.OST = 1;                                        // 解除整流右桥臂One-Shot Trip关断锁存
        rectifier_pwm_start_stage = 3;                                      // 标记主动整流PWM已经进入正常运行阶段
    }
    else if(rectifier_pwm_start_stage != 3)                                 // 非法阶段值下不允许主动整流输出
    {
        EPwm4Regs.TZFRC.bit.OST = 1;                                        // 异常状态保持整流左桥臂关断
        EPwm5Regs.TZFRC.bit.OST = 1;                                        // 异常状态保持整流右桥臂关断
    }
    EPwm6Regs.TZFRC.bit.OST = 1;                                            // ePWM6在全部模式下始终保持Trip关断
    EDIS;                                                                   // 重新禁止访问受保护寄存器
}

//******************* ePWM统一初始化 *******************//
void InitEPWM(void)                                                         // 冻结时基并统一配置所有ePWM模块
{
    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;                                   // 冻结全部ePWM时基以便同步配置
    EDIS;                                                                   // 重新禁止访问受保护寄存器

    EPWM1_Init();                                                           // 配置三相逆变A相ePWM
    EPWM2_Init();                                                           // 配置三相逆变B相ePWM
    EPWM3_Init();                                                           // 配置三相逆变C相ePWM及ADC触发
    EPWM4_Init();                                                           // 配置单相整流左桥臂ePWM
    EPWM5_Init();                                                           // 配置单相整流右桥臂ePWM
    EPWM6_Init();                                                           // 配置未使用的ePWM6并保持关断
    PWM_ForceAllLow();                                                      // 立即将所有ePWM输出强制为低电平
}

void EPWM1_Init(void)                                                       // 初始化ePWM1
{
    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;                                       // 打开对应ePWM模块外设时钟
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM1A为低
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM1B为低
    EPwm1Regs.TZFRC.bit.OST = 1;                                            // 初始化期间立即锁存关断ePWM1输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    EPwm1Regs.AQSFRC.bit.RLDCSF = 3;                                        // AQ连续软件强制使用立即模式，仅用于清除可能残留的旧状态
    EPwm1Regs.AQCSFRC.bit.CSFA = 0;                                         // 禁用旧的ePWM1A AQ连续软件强制
    EPwm1Regs.AQCSFRC.bit.CSFB = 0;                                         // 禁用旧的ePWM1B AQ连续软件强制

    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;                             // 选择该ePWM的同步输出来源
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;                                 // 配置相位同步装载功能
    EPwm1Regs.TBPHS.all = 0;                                                // 将同步相位偏移清零
    EPwm1Regs.TBCTR = 0;                                                    // 将时基计数器清零
    EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;                                     // 设置中心对齐PWM周期值
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;                          // 采用上下计数的中心对齐模式
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                                // 高速时基时钟不分频
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;                                   // 时基时钟不分频
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                             // 使能CMPA影子寄存器
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                             // 使能CMPB影子寄存器
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPA
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPB
    EPwm1Regs.CMPA.bit.CMPA = 0;                                            // 设置A路比较值
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;                                      // 计数器归零时将A路输出置高
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;                                    // 上数到CMPA时将A路输出清低
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;                                      // 下数到CMPA时将A路输出置高
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;                                      // 计数器归零时将B路原始输出置高
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;                                    // 上数到CMPB时将B路原始输出清低
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;                                      // 下数到CMPB时将B路原始输出置高
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;                                        // 选择ePWM A作为死区双沿输入
    EPwm1Regs.DBCTL.bit.POLSEL = 2;                                         // 设置互补输出极性
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;                                       // 同时使能上升沿和下降沿死区
    EPwm1Regs.DBRED.bit.DBRED = 15;                                         // 设置上升沿死区为15个TBCLK
    EPwm1Regs.DBFED.bit.DBFED = 15;                                         // 设置下降沿死区为15个TBCLK
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                               // 选择计数器归零作为ePWM中断事件
    EPwm1Regs.ETSEL.bit.INTEN = 0;                                          // 关闭未使用的ePWM中断请求
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;                                     // 配置每次事件产生一次中断标志
}

void EPWM2_Init(void)                                                       // 初始化ePWM2
{
    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;                                       // 打开对应ePWM模块外设时钟
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM2A为低
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM2B为低
    EPwm2Regs.TZFRC.bit.OST = 1;                                            // 初始化期间立即锁存关断ePWM2输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    EPwm2Regs.AQSFRC.bit.RLDCSF = 3;                                        // AQ连续软件强制使用立即模式，仅用于清除可能残留的旧状态
    EPwm2Regs.AQCSFRC.bit.CSFA = 0;                                         // 禁用旧的ePWM2A AQ连续软件强制
    EPwm2Regs.AQCSFRC.bit.CSFB = 0;                                         // 禁用旧的ePWM2B AQ连续软件强制

    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;                              // 选择该ePWM的同步输出来源
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;                                  // 配置相位同步装载功能
    EPwm2Regs.TBPHS.all = 0;                                                // 将同步相位偏移清零
    EPwm2Regs.TBCTR = 0;                                                    // 将时基计数器清零
    EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;                                     // 设置中心对齐PWM周期值
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;                          // 采用上下计数的中心对齐模式
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                                // 高速时基时钟不分频
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;                                   // 时基时钟不分频
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                             // 使能CMPA影子寄存器
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                             // 使能CMPB影子寄存器
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPA
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPB
    EPwm2Regs.CMPA.bit.CMPA = 0;                                            // 设置A路比较值
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;                                      // 计数器归零时将A路输出置高
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;                                    // 上数到CMPA时将A路输出清低
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;                                      // 下数到CMPA时将A路输出置高
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;                                      // 计数器归零时将B路原始输出置高
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;                                    // 上数到CMPB时将B路原始输出清低
    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;                                      // 下数到CMPB时将B路原始输出置高
    EPwm2Regs.DBCTL.bit.IN_MODE = 0;                                        // 选择ePWM A作为死区双沿输入
    EPwm2Regs.DBCTL.bit.POLSEL = 2;                                         // 设置互补输出极性
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;                                       // 同时使能上升沿和下降沿死区
    EPwm2Regs.DBRED.bit.DBRED = 15;                                         // 设置上升沿死区为15个TBCLK
    EPwm2Regs.DBFED.bit.DBFED = 15;                                         // 设置下降沿死区为15个TBCLK
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                               // 选择计数器归零作为ePWM中断事件
    EPwm2Regs.ETSEL.bit.INTEN = 0;                                          // 关闭未使用的ePWM中断请求
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;                                     // 配置每次事件产生一次中断标志
}

void EPWM3_Init(void)                                                       // 初始化ePWM3及SOCA触发
{
    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;                                       // 打开对应ePWM模块外设时钟
    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM3A为低
    EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM3B为低
    EPwm3Regs.TZFRC.bit.OST = 1;                                            // 初始化期间立即锁存关断ePWM3输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    EPwm3Regs.AQSFRC.bit.RLDCSF = 3;                                        // AQ连续软件强制使用立即模式，仅用于清除可能残留的旧状态
    EPwm3Regs.AQCSFRC.bit.CSFA = 0;                                         // 禁用旧的ePWM3A AQ连续软件强制
    EPwm3Regs.AQCSFRC.bit.CSFB = 0;                                         // 禁用旧的ePWM3B AQ连续软件强制

    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;                              // 选择该ePWM的同步输出来源
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;                                  // 配置相位同步装载功能
    EPwm3Regs.TBPHS.all = 0;                                                // 将同步相位偏移清零
    EPwm3Regs.TBCTR = 0;                                                    // 将时基计数器清零
    EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;                                     // 设置中心对齐PWM周期值
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;                          // 采用上下计数的中心对齐模式
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                                // 高速时基时钟不分频
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;                                   // 时基时钟不分频
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                             // 使能CMPA影子寄存器
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                             // 使能CMPB影子寄存器
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPA
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPB
    EPwm3Regs.CMPA.bit.CMPA = 0;                                            // 设置A路比较值
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;                                      // 计数器归零时将A路输出置高
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;                                    // 上数到CMPA时将A路输出清低
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;                                      // 下数到CMPA时将A路输出置高
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;                                      // 计数器归零时将B路原始输出置高
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;                                    // 上数到CMPB时将B路原始输出清低
    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;                                      // 下数到CMPB时将B路原始输出置高
    EPwm3Regs.DBCTL.bit.IN_MODE = 0;                                        // 选择ePWM A作为死区双沿输入
    EPwm3Regs.DBCTL.bit.POLSEL = 2;                                         // 设置互补输出极性
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;                                       // 同时使能上升沿和下降沿死区
    EPwm3Regs.DBRED.bit.DBRED = 15;                                         // 设置上升沿死区为15个TBCLK
    EPwm3Regs.DBFED.bit.DBFED = 15;                                         // 设置下降沿死区为15个TBCLK
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                               // 选择计数器归零作为ePWM中断事件
    EPwm3Regs.ETSEL.bit.INTEN = 0;                                          // 关闭未使用的ePWM中断请求
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;                                     // 配置每次事件产生一次中断标志
    EPwm3Regs.ETSEL.bit.SOCAEN = 1;                                         // 使能ePWM3的ADC启动脉冲
    EPwm3Regs.ETSEL.bit.SOCASEL = 1;                                        // 选择TBCTR等于零作为ADC采样时刻
    EPwm3Regs.ETPS.bit.SOCAPRD = 1;                                         // 每次采样事件都产生SOCA
}

void EPWM4_Init(void)                                                       // 初始化整流左桥臂ePWM4
{
    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;                                       // 打开对应ePWM模块外设时钟
    EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM4A为低
    EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM4B为低
    EPwm4Regs.TZFRC.bit.OST = 1;                                            // 初始化期间立即锁存关断ePWM4输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    EPwm4Regs.AQSFRC.bit.RLDCSF = 3;                                        // AQ连续软件强制使用立即模式，仅用于清除可能残留的旧状态
    EPwm4Regs.AQCSFRC.bit.CSFA = 0;                                         // 禁用旧的ePWM4A AQ连续软件强制
    EPwm4Regs.AQCSFRC.bit.CSFB = 0;                                         // 禁用旧的ePWM4B AQ连续软件强制

    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;                             // 选择该ePWM的同步输出来源
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;                                 // 配置相位同步装载功能
    EPwm4Regs.TBPHS.all = 0;                                                // 将同步相位偏移清零
    EPwm4Regs.TBCTR = 0;                                                    // 将时基计数器清零
    EPwm4Regs.TBPRD = EPWM_TIMER_TBPRD;                                     // 设置中心对齐PWM周期值
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;                          // 采用上下计数的中心对齐模式
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                                // 高速时基时钟不分频
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;                                   // 时基时钟不分频
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                             // 使能CMPA影子寄存器
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                             // 使能CMPB影子寄存器
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPA
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPB
    EPwm4Regs.CMPA.bit.CMPA = 0;                                            // 设置A路比较值
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;                                      // 计数器归零时将A路输出置高
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;                                    // 上数到CMPA时将A路输出清低
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;                                      // 下数到CMPA时将A路输出置高
    EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;                                      // 计数器归零时将B路原始输出置高
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;                                    // 上数到CMPB时将B路原始输出清低
    EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;                                      // 下数到CMPB时将B路原始输出置高
    EPwm4Regs.DBCTL.bit.IN_MODE = 0;                                        // 选择ePWM A作为死区双沿输入
    EPwm4Regs.DBCTL.bit.POLSEL = 2;                                         // 设置互补输出极性
    EPwm4Regs.DBCTL.bit.OUT_MODE = 3;                                       // 同时使能上升沿和下降沿死区
    EPwm4Regs.DBRED.bit.DBRED = 15;                                         // 设置上升沿死区为15个TBCLK
    EPwm4Regs.DBFED.bit.DBFED = 15;                                         // 设置下降沿死区为15个TBCLK
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                               // 选择计数器归零作为ePWM中断事件
    EPwm4Regs.ETSEL.bit.INTEN = 0;                                          // 关闭未使用的ePWM中断请求
    EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;                                     // 配置每次事件产生一次中断标志
}

void EPWM5_Init(void)                                                       // 初始化整流右桥臂ePWM5
{
    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;                                       // 打开对应ePWM模块外设时钟
    EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM5A为低
    EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM5B为低
    EPwm5Regs.TZFRC.bit.OST = 1;                                            // 初始化期间立即锁存关断ePWM5输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    EPwm5Regs.AQSFRC.bit.RLDCSF = 3;                                        // AQ连续软件强制使用立即模式，仅用于清除可能残留的旧状态
    EPwm5Regs.AQCSFRC.bit.CSFA = 0;                                         // 禁用旧的ePWM5A AQ连续软件强制
    EPwm5Regs.AQCSFRC.bit.CSFB = 0;                                         // 禁用旧的ePWM5B AQ连续软件强制

    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;                              // 继续传递来自ePWM4的同步信号
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;                                  // 使能相位装载以便与ePWM4使用同相载波
    EPwm5Regs.TBPHS.all = 0;                                                // 将同步相位偏移清零
    EPwm5Regs.TBCTR = 0;                                                    // 将时基计数器清零
    EPwm5Regs.TBPRD = EPWM_TIMER_TBPRD;                                     // 设置中心对齐PWM周期值
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;                          // 采用上下计数的中心对齐模式
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                                // 高速时基时钟不分频
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;                                   // 时基时钟不分频
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                             // 使能CMPA影子寄存器
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                             // 使能CMPB影子寄存器
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPA
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPB
    EPwm5Regs.CMPA.bit.CMPA = 0;                                            // 设置A路比较值
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;                                      // 计数器归零时将A路输出置高
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;                                    // 上数到CMPA时将A路输出清低
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;                                      // 下数到CMPA时将A路输出置高
    EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;                                      // 计数器归零时将B路原始输出置高
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;                                    // 上数到CMPB时将B路原始输出清低
    EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;                                      // 下数到CMPB时将B路原始输出置高
    EPwm5Regs.DBCTL.bit.IN_MODE = 0;                                        // 选择ePWM A作为死区双沿输入
    EPwm5Regs.DBCTL.bit.POLSEL = 2;                                         // 设置互补输出极性
    EPwm5Regs.DBCTL.bit.OUT_MODE = 3;                                       // 同时使能上升沿和下降沿死区
    EPwm5Regs.DBRED.bit.DBRED = 15;                                         // 设置上升沿死区为15个TBCLK
    EPwm5Regs.DBFED.bit.DBFED = 15;                                         // 设置下降沿死区为15个TBCLK
    EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                               // 选择计数器归零作为ePWM中断事件
    EPwm5Regs.ETSEL.bit.INTEN = 0;                                          // 关闭未使用的ePWM中断请求
    EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;                                     // 配置每次事件产生一次中断标志
}

void EPWM6_Init(void)                                                       // 初始化未使用的ePWM6
{
    EALLOW;                                                                 // 允许访问受保护寄存器
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;                                       // 打开对应ePWM模块外设时钟
    EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM6A为低
    EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO;                                  // One-Shot Trip发生时从死区模块之后强制ePWM6B为低
    EPwm6Regs.TZFRC.bit.OST = 1;                                            // 初始化期间立即锁存关断未使用的ePWM6输出
    EDIS;                                                                   // 重新禁止访问受保护寄存器
    EPwm6Regs.AQSFRC.bit.RLDCSF = 3;                                        // AQ连续软件强制使用立即模式，仅用于清除可能残留的旧状态
    EPwm6Regs.AQCSFRC.bit.CSFA = 0;                                         // 禁用旧的ePWM6A AQ连续软件强制
    EPwm6Regs.AQCSFRC.bit.CSFB = 0;                                         // 禁用旧的ePWM6B AQ连续软件强制

    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;                              // 选择该ePWM的同步输出来源
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;                                  // 配置相位同步装载功能
    EPwm6Regs.TBPHS.all = 0;                                                // 将同步相位偏移清零
    EPwm6Regs.TBCTR = 0;                                                    // 将时基计数器清零
    EPwm6Regs.TBPRD = EPWM_TIMER_TBPRD;                                     // 设置中心对齐PWM周期值
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;                          // 采用上下计数的中心对齐模式
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                                // 高速时基时钟不分频
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;                                   // 时基时钟不分频
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;                             // 使能CMPA影子寄存器
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;                             // 使能CMPB影子寄存器
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPA
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;                           // 在计数器归零时装载CMPB
    EPwm6Regs.CMPA.bit.CMPA = 0;                                            // 设置A路比较值
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;                                      // 计数器归零时将A路输出置高
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;                                    // 上数到CMPA时将A路输出清低
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;                                      // 下数到CMPA时将A路输出置高
    EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;                                      // 计数器归零时将B路原始输出置高
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;                                    // 上数到CMPB时将B路原始输出清低
    EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;                                      // 下数到CMPB时将B路原始输出置高
    EPwm6Regs.DBCTL.bit.IN_MODE = 0;                                        // 选择ePWM A作为死区双沿输入
    EPwm6Regs.DBCTL.bit.POLSEL = 2;                                         // 设置互补输出极性
    EPwm6Regs.DBCTL.bit.OUT_MODE = 3;                                       // 同时使能上升沿和下降沿死区
    EPwm6Regs.DBRED.bit.DBRED = 15;                                         // 设置上升沿死区为15个TBCLK
    EPwm6Regs.DBFED.bit.DBFED = 15;                                         // 设置下降沿死区为15个TBCLK
    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                               // 选择计数器归零作为ePWM中断事件
    EPwm6Regs.ETSEL.bit.INTEN = 0;                                          // 关闭未使用的ePWM中断请求
    EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;                                     // 配置每次事件产生一次中断标志
}
