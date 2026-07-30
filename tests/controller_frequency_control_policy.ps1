$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\controller.c')

foreach ($required in @(
    '#define OUTPUT_FREQ_LOW_HZ 30.0f',
    '#define OUTPUT_FREQ_HIGH_HZ 60.0f',
    '#define OUTPUT_FREQ_RAMP_STEP_HZ 0.00151f',
    '#define FREQ_KEY_SCAN_ISR_DIV 100',
    'volatile float output_freq_target_hz = OUTPUT_FREQ_HIGH_HZ;',
    'volatile float output_freq_actual_hz = OUTPUT_FREQ_HIGH_HZ;',
    'theta_a += 2.0f * pi * output_freq_actual_hz * Ts;',
    'output_freq_target_hz = OUTPUT_FREQ_HIGH_HZ;',
    'output_freq_target_hz = OUTPUT_FREQ_LOW_HZ;',
    'output_freq_actual_hz += OUTPUT_FREQ_RAMP_STEP_HZ;',
    'output_freq_actual_hz -= OUTPUT_FREQ_RAMP_STEP_HZ;'
)) {
    if (-not $source.Contains($required)) {
        throw "controller frequency policy missing: $required"
    }
}

$key1 = [regex]::Match($source, '(?s)case KEY1_PRESS:.*?break;')
if (-not $key1.Success -or
    $key1.Value -notmatch 'output_freq_target_hz = OUTPUT_FREQ_HIGH_HZ' -or
    $key1.Value -match '\+\=|U_BUS_REF') {
    throw 'KEY1 must directly select 60 Hz without changing another reference.'
}

$key2 = [regex]::Match($source, '(?s)case KEY2_PRESS:.*?break;')
if (-not $key2.Success -or
    $key2.Value -notmatch 'output_freq_target_hz = OUTPUT_FREQ_LOW_HZ' -or
    $key2.Value -match '\-\=|U_BUS_REF') {
    throw 'KEY2 must directly select 30 Hz without changing another reference.'
}

$step = [single]0.00151
$up = [single]30.0
$down = [single]60.0
for ($sample = 0; $sample -lt 20000; $sample++) {
    $up = [single]($up + $step); if($up -gt [single]60.0) { $up = [single]60.0 }
    $down = [single]($down - $step); if($down -lt [single]30.0) { $down = [single]30.0 }
}
if ([Math]::Abs($up - 60.0) -gt 1.0e-9 -or
    [Math]::Abs($down - 30.0) -gt 1.0e-9 -or
    (100 + [Math]::Ceiling(30.0 / $step)) -gt 20000) {
    throw "30 Hz transition does not finish in 20000 ISR samples: up=$up, down=$down"
}

'controller 30/60 Hz one-second ramp policy: PASS'
