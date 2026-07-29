#ifndef ADC_AVERAGE_H
#define ADC_AVERAGE_H

typedef struct
{
    Uint32 sum;                                                             // 当前平均窗口内的原始ADC累加值
    Uint16 count;                                                           // 当前平均窗口已累加的采样点数
    Uint16 window;                                                          // 当前窗口长度，改变时自动放弃未完成窗口
    float average;                                                          // 最近一个完整窗口的平均值
} ADC_AverageState;                                                         // 每个ADC通道应使用一个独立状态

static inline float ADC_Average_Update(ADC_AverageState *state, Uint16 sample,
                                      Uint16 sample_count)                  // 更新任意ADC RESULT的分段平均值
{
    if(state == (ADC_AverageState *)0)                                      // 通用接口防止空状态指针访问
        return 0.0f;
    if(sample_count == 0U)                                                  // 零窗口无效，保持已有结果和累加状态
        return state->average;

    if((state->count != 0U) && (state->window != sample_count))             // 未完成窗口中途改变长度时禁止混合旧数据
    {
        state->sum = 0UL;                                                    // 丢弃旧窗口累加值
        state->count = 0U;                                                   // 从本次采样开始建立新窗口
    }
    state->window = sample_count;                                            // 记录当前通道使用的窗口长度
    state->sum += (Uint32)sample;                                            // 累加本次传入的ADC原始结果
    state->count++;
    if(state->count >= sample_count)                                        // 第sample_count点到达时更新平均值
    {
        state->average = (float)state->sum / (float)sample_count;
        state->sum = 0UL;
        state->count = 0U;
    }
    return state->average;                                                  // 窗口未满时返回上一个完整窗口结果
}

#endif
