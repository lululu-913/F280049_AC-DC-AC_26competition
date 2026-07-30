# controller.c 低功耗修改说明

## 1. 修改目标

在不改变整流、逆变控制频率和保护逻辑的前提下，降低 F280049 DSP 及显示部分的功耗，尽量提高整机效率。

本说明优先处理以下项目：

1. 关闭未使用的外设时钟。
2. 删除未使用的 ePWM6 配置。
3. 主循环空闲时让 CPU 进入 IDLE。
4. 降低 OLED 刷新带来的 CPU 和显示功耗。
5. 后续再考虑减少停机状态下的中断计算。

建议每完成一项就单独编译、烧录和验证，不要一次性完成全部修改。

---

## 2. 修改前准备

### 2.1 备份当前可工作的代码

修改前复制一份当前能够正常整流和逆变的 `controller.c`，或者建立一个单独的 Git 提交。

建议记录修改前基准数据：

| 测试项目 | 修改前数据 |
|---|---:|
| 辅助电源输入功率 |  |
| 36 V 输入时整机输入功率 |  |
| 60 Hz 输出效率 |  |
| 30 Hz 输出效率 |  |
| 输入功率因数 |  |
| 输出线电压 THD |  |
| DSP 温度或板上温升 |  |

### 2.2 修改原则

- ePWM1～3 是逆变部分，不能关闭。
- ePWM4～5 是整流部分，不能关闭。
- ePWM4 还负责触发 ADC，不能停止其时基。
- ADCA、ADCB 不能关闭。
- ADCC 暂时保留，确认不会影响 ADCB 参考和采样噪声后才能试验关闭。
- `TBCLKSYNC` 最终必须恢复为 1。
- PIE、CPU 中断和 ADCA 中断必须保持工作。

---

## 3. 第一阶段：关闭未使用的外设时钟

这是优先级最高、风险相对最低的修改。

### 3.1 当前问题

在 `controller.c` 中搜索：

```c
InitSysCtrl();
```

TI 提供的 `InitSysCtrl()` 会调用 `InitPeripheralClocks()`，默认打开大量外设时钟，包括当前工程没有使用的：

- CLA、DMA
- CPU Timer0～2
- HRPWM
- ePWM6～8
- eCAP1～7
- eQEP1～2
- SDFM
- SCI-A/B
- SPI-A/B
- I2C-A
- CAN-A/B
- CMPSS1～7
- PGA1～7
- FSI RX/TX
- DAC-A/B
- LIN、PMBus、DCC

OLED 使用 GPIO 模拟通信，并不依赖硬件 I2C-A。

### 3.2 推荐修改方法

不要直接大范围改动 TI 的 `f28004x_sysctrl.c`。在 `controller.c` 中增加一个本工程专用的时钟精简函数，便于恢复和维护。

在函数声明区域加入：

```c
static void ConfigureUsedPeripheralClocks(void);
```

在 `controller.c` 的初始化辅助函数区域加入：

```c
static void ConfigureUsedPeripheralClocks(void)
{
    /*
     * InitSysCtrl() 已完成系统时钟、Flash 和器件校准。
     * 此处只关闭外设时钟，然后重新打开本工程所需模块。
     */
    DisablePeripheralClocks();

    EALLOW;

    /*
     * ADC-A、ADC-B 是控制采样所必需的。
     * ADC-C 当前虽然没有配置 SOC，但原代码用于稳定共享参考，
     * 第一阶段先保留，完成实测后再决定是否关闭。
     */
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1;

    EDIS;
}
```

然后把初始化流程从：

```c
InitSysCtrl();
InitGpio();
```

改为：

```c
InitSysCtrl();
ConfigureUsedPeripheralClocks();
InitGpio();
```

### 3.3 ePWM 时钟的处理

检查 `EPWM1_Init()` 至 `EPWM5_Init()` 内部是否分别包含类似代码：

```c
CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
```

如果 ePWM1～5 的初始化函数都会自行打开相应时钟，就不需要在 `ConfigureUsedPeripheralClocks()` 中重复开启。

如果某个 ePWM 初始化函数没有打开自己的时钟，则应在调用该初始化函数之前显式打开。

### 3.4 必须检查 TBCLKSYNC

搜索：

```c
TBCLKSYNC
```

初始化 ePWM 前通常需要：

```c
CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
```

全部 ePWM 配置完成后必须恢复：

```c
CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
```

如果没有恢复，所有 ePWM 时基都会停止，整流和逆变均无法启动。

### 3.5 第一阶段验证

烧录后依次确认：

1. OLED能够正常显示。
2. 按键能够正常识别。
3. ADCA中断持续进入。
4. ePWM1～5频率与修改前一致。
5. 逆变能够独立启动。
6. 整流能够独立启动。
7. 整流和逆变同时运行正常。
8. F1、F2和过流保护仍能触发。

如果程序在 `InitADC()` 附近停止，优先检查 ADC-A/B/C 时钟是否在调用 `InitADC()` 前开启。

---

## 4. 第二阶段：彻底移除 ePWM6

### 4.1 当前情况

代码注释说明 ePWM6 不参与功率变换，但目前仍然存在：

- GPIO10、GPIO11 的 ePWM6 复用配置。
- `EPWM6_Init()` 调用。
- ePWM6 外设时钟。
- ePWM6 时基、死区和动作限定配置。
- 停机函数中对 `EPwm6Regs` 的 Trip 操作。

### 4.2 修改步骤

#### 步骤一：确认硬件

先确认 GPIO10、GPIO11 没有连接到必须控制的驱动、继电器或保护电路。

#### 步骤二：删除初始化调用

搜索并删除或注释：

```c
EPWM6_Init();
```

同时删除 `EPWM6_Init()` 的函数声明和函数定义。

#### 步骤三：删除 ePWM6 Trip 操作

搜索：

```c
EPwm6Regs
```

删除只用于 ePWM6 的 Trip、清除和比较寄存器写入。不要误删 ePWM1～5 的保护操作。

#### 步骤四：GPIO10、GPIO11改为安全低电平

删除 GPIO10、GPIO11 的 ePWM6 复用，改成普通输出低电平。示意代码如下，具体寄存器写法以项目现有 GPIO 初始化风格为准：

```c
EALLOW;

GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;

GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;

EDIS;
```

注意：不同 GPIO 可能还需要同时清除 `GPAGMUX1` 对应位。建议优先使用项目中已有的 GPIO 配置函数。

### 4.3 验证

- 示波器确认 GPIO10、GPIO11始终为低电平。
- 确认逆变 ePWM1～3未受影响。
- 确认整流 ePWM4～5未受影响。
- 重新检查所有 Trip 保护。

---

## 5. 第三阶段：主循环增加 IDLE

### 5.1 当前问题

主循环完成 OLED 刷新、参数限制等后台任务后会立即再次循环，CPU持续执行空循环。

由于控制主要在 ADCA 中断中执行，可以让 CPU 在两次控制中断之间进入空闲状态。

### 5.2 修改方法

在 `main()` 中搜索：

```c
while(1)
```

把 `IDLE` 放在所有后台任务之后、循环结束之前：

```c
while(1)
{
    /*
     * 原有OLED、按键后台处理和参数限制代码保持不变。
     */

    asm(" IDLE");
}
```

ADCA中断到来后CPU会自动退出IDLE并执行控制中断。

### 5.3 注意事项

- 必须确保全局中断已经开启。
- 必须确保 ePWM4 的 ADC 触发始终运行。
- Trip只关闭PWM输出，不应停止ePWM4时基。
- 如果程序需要在未启动PWM时也执行后台任务，要确认ePWM4计数器在待机状态仍运行。
- 在线调试时，单步执行经过IDLE可能表现得像程序暂停，应以中断计数变量确认程序是否工作。

### 5.4 验证

增加一个仅供调试的中断计数变量：

```c
volatile uint32_t adc_isr_count = 0;
```

在ADCA中断入口增加：

```c
adc_isr_count++;
```

确认加入IDLE后该计数仍持续增加，并且输出频率、THD和动态响应没有变化。验证完成后可以删除该调试变量。

---

## 6. 第四阶段：降低 OLED 功耗

OLED既消耗显示电流，也通过GPIO模拟通信占用CPU时间。

### 6.1 降低刷新频率

搜索：

```c
N_oled
```

当前刷新机制大约每50 ms更新一行、约200 ms更新整屏。可以先把整屏更新周期放慢至500 ms左右。

修改时不要改变控制中断周期，只增大OLED刷新分频计数。

### 6.2 只在数据变化时刷新

可保存上一次显示值，只有显示整数或状态发生变化时才调用OLED写函数。例如：

```c
static uint16_t last_fault_code = 0xFFFFU;

if (fault_code != last_fault_code)
{
    last_fault_code = fault_code;
    /* 更新对应OLED区域 */
}
```

电压、电流显示也可以先量化到0.1 V或0.1 A，量化结果变化后再刷新。

### 6.3 增加统一开关

在宏定义区域增加：

```c
#define OLED_ENABLE 1U
```

初始化和刷新处分别写成：

```c
#if OLED_ENABLE
    OLED_Init();
#endif
```

```c
#if OLED_ENABLE
    OLED_UpdateOneRow();
#endif
```

函数名称应替换为当前工程实际使用的OLED函数。

比赛测效率时可以暂时设为：

```c
#define OLED_ENABLE 0U
```

如果OLED供电仍然存在，仅停止通信不一定能关闭屏幕电流。需要调用 `oled.c` 中现有的显示关闭函数，或者关闭OLED供电。

---

## 7. 第五阶段：减少停机时的中断计算

这一阶段会改动控制流程，风险高于前面几项，应最后进行。

### 7.1 可跳过的运算

逆变未运行时，通常可以跳过：

- 三相输出电压重构。
- 输出RMS计算。
- 相间平衡计算。
- 输出电压环和调制计算。
- 仅用于显示的输出滤波。

示意结构：

```c
if (inverter_enabled)
{
    InverterMeasurementUpdate();
    InverterControlUpdate();
}
else
{
    /*
     * 保留必要的零点更新、故障检查和安全状态处理。
     */
}
```

### 7.2 PLL处理

当前PLL持续运行，可以让整流启动时立即获得锁相角度。这对启动可靠性有利。

不建议第一轮就降低PLL计算频率。如果确实需要进一步降低待机功耗，可以只在整流关闭时每2～4个中断更新一次PLL，但必须重新验证：

- 整流启动时间。
- 输入电流相位。
- 功率因数。
- 启动冲击电流。

---

## 8. 可选项目：减少重复Trip寄存器写入

当前逆变或整流处于关闭状态时，可能在每次控制中断中重复调用：

```c
PWM_TripRectifier();
PWM_TripInverter();
```

OST是锁存保护，理论上在状态切换时写一次即可。

可以增加上一次状态：

```c
static uint16_t rectifier_was_enabled = 0U;
static uint16_t inverter_was_enabled = 0U;
```

在状态由运行变为关闭时执行Trip：

```c
if ((!rectifier_enabled) && rectifier_was_enabled)
{
    PWM_TripRectifier();
}

if ((!inverter_enabled) && inverter_was_enabled)
{
    PWM_TripInverter();
}

rectifier_was_enabled = rectifier_enabled;
inverter_was_enabled = inverter_enabled;
```

此项不能削弱故障保护。硬件故障、软件过压和软件过流路径仍必须立即执行Trip。

只有在确认没有任何代码会在关闭状态意外清除OST后，才能取消中断中的重复Trip。

---

## 9. ADCC关闭试验

ADCC没有配置实际SOC，但原代码说明其上电可能用于稳定ADCB/ADCC共享参考，因此不能直接删除。

完成前面修改并保证系统稳定后，可以进行A/B测试：

### 9.1 A组

保持：

```c
CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
```

记录：

- ADCB零点。
- ADCB峰峰值噪声。
- 输入电流波形。
- 输入功率因数。
- F1/F2误触发次数。

### 9.2 B组

关闭ADCC：

```c
CpuSysRegs.PCLKCR13.bit.ADC_C = 0;
```

在相同输入、负载和温度条件下重复测量。

只有当两组数据没有明显差异时，才可以永久关闭ADCC。

---

## 10. 不建议修改的项目

### 10.1 暂时不要降低SYSCLK

降低100 MHz系统时钟会同时影响：

- ePWM的 `TBPRD`。
- 死区时间。
- ADC时钟和采样窗口。
- 中断执行时间。
- 软件延时函数。
- OLED通信时序。

除非准备重新计算全部时序参数，否则不建议通过降主频降低功耗。

### 10.2 不要关闭核心控制资源

以下资源不能关闭：

- ADCA、ADCB。
- ePWM1～5。
- ePWM时基同步。
- ADCA中断。
- PIE和CPU全局中断。
- 功率器件保护相关GPIO。

---

## 11. 推荐实施顺序

### 第一次修改

- 增加 `ConfigureUsedPeripheralClocks()`。
- 关闭未使用的外设时钟。
- 暂时保留ADCC。

验证全部功能后再继续。

### 第二次修改

- 删除ePWM6。
- GPIO10、GPIO11改为安全低电平。

验证全部PWM和保护后再继续。

### 第三次修改

- 主循环增加IDLE。

验证ADCA中断、OLED和按键后再继续。

### 第四次修改

- OLED降低刷新率。
- 增加 `OLED_ENABLE`。
- 测效率时关闭显示。

### 第五次修改

- 停机状态跳过不必要计算。
- 根据测试结果决定是否优化重复Trip。
- 最后试验是否能够关闭ADCC。

---

## 12. 每一步的验收表

| 项目 | 修改前 | 修改后 | 是否通过 |
|---|---:|---:|---|
| 整流ePWM频率 |  |  |  |
| 逆变ePWM频率 |  |  |  |
| 60 Hz输出频率 |  |  |  |
| 30 Hz输出频率 |  |  |  |
| 60 Hz线电压 |  |  |  |
| 30 Hz线电压 |  |  |  |
| 输入功率因数 |  |  |  |
| 输出THD |  |  |  |
| 整机效率 |  |  |  |
| F1保护 |  |  |  |
| F2保护 |  |  |  |
| 过流保护 |  |  |  |
| 启动时间 |  |  |  |
| 辅助电源功率 |  |  |  |

如果某一步出现异常，只回退该步骤，不要同时调整控制环参数。低功耗修改不应改变电压环、电流环、PLL、母线前馈和保护阈值。
