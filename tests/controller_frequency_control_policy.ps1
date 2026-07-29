$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    '#define OUTPUT_FREQ_MIN_HZ 45U',
    '#define OUTPUT_FREQ_MAX_HZ 505U',
    '#define OUTPUT_FREQ_STEP_HZ 1U',
    '#define FREQ_KEY_REPEAT_ISR_DIV 5000',
    '#define GENERAL_KEY_SCAN_ISR_DIV 10000',
    'volatile Uint16 output_freq_hz = 50U;',
    'volatile float U_BUS_REF = 55.0f;',
    'theta_a += 2.0f * pi * (float)output_freq_hz * Ts;',
    'if(output_freq_hz < OUTPUT_FREQ_MAX_HZ)',
    'output_freq_hz += OUTPUT_FREQ_STEP_HZ;',
    'if(output_freq_hz > OUTPUT_FREQ_MIN_HZ)',
    'output_freq_hz -= OUTPUT_FREQ_STEP_HZ;'
)) {
    if (-not $source.Contains($required)) {
        throw "controller frequency policy missing: $required"
    }
}

$frequencyTimer = [regex]::Match(
    $source,
    '(?s)N_freq_key\+\+;.*?if\(N_freq_key >= FREQ_KEY_REPEAT_ISR_DIV\).*?\n\s*\}')
if (-not $frequencyTimer.Success -or
    $frequencyTimer.Value -notmatch 'KEY_H1' -or
    $frequencyTimer.Value -notmatch 'KEY_H2') {
    throw 'controller frequency policy: KEY1/KEY2 lack an independent 250 ms repeat timer'
}

$generalTimer = [regex]::Match(
    $source,
    '(?s)N_key\+\+;.*?if\(N_key >= GENERAL_KEY_SCAN_ISR_DIV\).*?\n\s*\}')
if (-not $generalTimer.Success -or
    $generalTimer.Value -notmatch 'KEY_Scan') {
    throw 'controller frequency policy: KEY3-KEY6 no longer retain their 500 ms scan timer'
}

$keyScan = [regex]::Match(
    $source,
    '(?s)char KEY_Scan\(char key_mode\).*?\n\}')
if (-not $keyScan.Success -or
    $keyScan.Value -match 'KEY_H1|KEY_H2') {
    throw 'controller frequency policy: general key scan would double-trigger KEY1/KEY2'
}

$key1 = [regex]::Match(
    $source,
    '(?s)case KEY1_PRESS:.*?break;')
if (-not $key1.Success -or
    $key1.Value -notmatch 'OUTPUT_FREQ_MAX_HZ' -or
    $key1.Value -match 'U_BUS_REF') {
    throw 'controller frequency policy: KEY1 is not dedicated to bounded frequency increase'
}

$key2 = [regex]::Match(
    $source,
    '(?s)case KEY2_PRESS:.*?break;')
if (-not $key2.Success -or
    $key2.Value -notmatch 'OUTPUT_FREQ_MIN_HZ' -or
    $key2.Value -match 'U_BUS_REF') {
    throw 'controller frequency policy: KEY2 is not dedicated to bounded frequency decrease'
}

'controller frequency control policy: PASS'
