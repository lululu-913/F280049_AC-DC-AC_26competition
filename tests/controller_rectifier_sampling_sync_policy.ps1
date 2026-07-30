$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\controller.c')

foreach ($required in @(
    '#define INVERTER_EPWM_TBPRD  2500',
    '#define RECTIFIER_EPWM_TBPRD 5000',
    '#define OUTPUT_FREQ_RAMP_STEP_HZ 0.00151f',
    '#define FREQ_KEY_SCAN_ISR_DIV 100',
    '#define GENERAL_KEY_SCAN_ISR_DIV 10000',
    '#define RECTIFIER_CURRENT_KI 0.01f',
    '#define RECTIFIER_CURRENT_INTEGRAL_LIMIT 30.0f',
    '#define OUTPUT_COMMON_IIR_ALPHA 0.00624f',
    '#define OUTPUT_PHASE_IIR_ALPHA 0.000628f',
    '#define BUS_FEEDFORWARD_IIR_ALPHA 0.086f',
    '#define OLED_REFRESH_DIVIDER 1000',
    'int N = 200;',
    'float Ts = 0.00005f;',
    'float kp = 100.0f;',
    'float ki = 10.0f;',
    'EPwm3Regs.ETSEL.bit.SOCAEN = 0;',
    'EPwm4Regs.ETSEL.bit.SOCAEN = 1;',
    'EPwm4Regs.ETSEL.bit.SOCASEL = 3;',
    'EPwm4Regs.ETPS.bit.SOCAPRD = 1;',
    'AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11;',
    'AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11;',
    'AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 11;',
    'AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 11;',
    'AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 11;',
    'EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;',
    'EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;',
    'EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;',
    'EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;',
    'EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;',
    'EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;',
    'EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;',
    'EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;',
    'EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;',
    'EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP;',
    'EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;',
    'EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;'
)) {
    if (-not $source.Contains($required)) { throw "Rectifier sampling sync policy missing: $required" }
}

$adcTriggerMatches = [regex]::Matches($source, 'Adc[ab]Regs\.ADCSOC[0-9]+CTL\.bit\.TRIGSEL\s*=\s*11;')
if ($adcTriggerMatches.Count -ne 5) {
    throw "Expected five ADC SOCs triggered by ePWM4 SOCA, found $($adcTriggerMatches.Count)."
}
if ($source -match 'ADCSOC[0-9]+CTL\.bit\.TRIGSEL\s*=\s*9;') {
    throw 'An ADC SOC still uses asynchronous ePWM3 SOCA.'
}
if (-not $source.Contains('AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;')) {
    throw 'ADCA EOC3 must remain the control ISR completion source.'
}

'controller ePWM4-synchronous 20kHz sampling policy: PASS'
