# ADCB0～B3 同步采样设计

## 目标

在不改变现有控制算法和ADC中断频率的前提下，启用F280049的ADCINB0～ADCINB3四个输入通道，使其每个20kHz控制周期同步更新对应的原始转换结果。

## 通道配置

在现有 `InitADCSOC()` 中配置：

- ADCB SOC0选择ADCINB0，结果写入`AdcbResultRegs.ADCRESULT0`。
- ADCB SOC1选择ADCINB1，结果写入`AdcbResultRegs.ADCRESULT1`。
- ADCB SOC2选择ADCINB2，结果写入`AdcbResultRegs.ADCRESULT2`。
- ADCB SOC3选择ADCINB3，结果写入`AdcbResultRegs.ADCRESULT3`。

四个SOC统一使用：

- `ACQPS = 9`，即10个SYSCLK采样保持窗口。
- `TRIGSEL = 9`，即由ePWM3 SOCA在20kHz同步触发。

## 中断与数据使用

继续使用ADCA EOC3触发现有ADCINT1和`adcA1ISR()`。ADCA与ADCB并行转换，ADCB SOC0～3采用相同采样窗口，因此ADCA EOC3发生时ADCB RESULT0～3已经完成更新。

现有ADCINB0输入电流换算保持不变。保留用户已经加入的ADCRESULT1累加平均观察逻辑，不修改其变量或计算周期。ADCINB2～3仅启用原始结果寄存器，不新增浮点换算变量，也不参与控制、保护或OLED显示；后续取得各通道零点和增益标定后再单独设计换算。

## 验证

- 策略测试断言ADCB SOC0～3分别映射到通道0～3。
- 策略测试断言四路均使用`ACQPS=9`和`TRIGSEL=9`。
- 运行全部现有策略测试。
- 使用CCS编译器完整编译和链接Debug固件。

## 非目标

本次不修改ADCA配置、ADC中断源、ePWM3触发频率、输入电流标定、PID参数或功率级控制逻辑。
