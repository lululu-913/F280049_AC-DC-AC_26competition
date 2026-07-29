$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

$initSocMatch = [regex]::Match(
    $source,
    '(?s)void\s+InitADCSOC\s*\(\s*void\s*\).*?\r?\n\{.*?\r?\n\}\s*(?=//\*+\s*ADC中断)')
if (-not $initSocMatch.Success) {
    throw 'ADCB channel policy cannot isolate InitADCSOC().'
}
$initSoc = $initSocMatch.Value

foreach ($soc in 0..3) {
    foreach ($field in @(
        "AdcbRegs.ADCSOC${soc}CTL.bit.CHSEL = $soc;",
        "AdcbRegs.ADCSOC${soc}CTL.bit.ACQPS = 9;",
        "AdcbRegs.ADCSOC${soc}CTL.bit.TRIGSEL = 9;"
    )) {
        if (-not $initSoc.Contains($field)) {
            throw "ADCB channel policy missing: $field"
        }
    }
}

if (-not $initSoc.Contains('AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;')) {
    throw 'ADCB channel policy must preserve ADCA EOC3 as the ADCINT1 source.'
}

$isrMatch = [regex]::Match(
    $source,
    '(?s)__interrupt\s+void\s+adcA1ISR\s*\(\s*void\s*\).*?\r?\n\{.*?(?=//\*+\s*锁相环)')
if (-not $isrMatch.Success) {
    throw 'ADCB channel policy cannot isolate adcA1ISR().'
}
$isr = $isrMatch.Value

if (-not $isr.Contains(
    'I_in = ((float)AdcbResultRegs.ADCRESULT0 - 2066.2f) / 164.61f;')) {
    throw 'ADCB channel policy must preserve the existing B0 current conversion.'
}

if (-not $isr.Contains(
    'av = ADC_Average_Update(&adcb_result0_average, AdcbResultRegs.ADCRESULT0, ADC_AVERAGE_WINDOW_SAMPLES);')) {
    throw 'ADCB channel policy must use the reusable average function for B0.'
}

foreach ($unusedChannel in 1..3) {
    if ($source -match "AdcbResultRegs\.ADCRESULT$unusedChannel\b") {
        throw "ADCB$unusedChannel must remain raw-only without a controller consumer."
    }
}

'PASS: ADCB0-B3 sample synchronously while B0 conversion and reusable averaging remain intact.'
