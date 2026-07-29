$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $path
$activeSource = [regex]::Replace(
    $source,
    '(?s)#if\s+!RECTIFIER_ONLY_MODE.*?#endif',
    '')

if ($source -notmatch '#define\s+RECTIFIER_ONLY_MODE\s+1') {
    throw 'rectifier-only policy: RECTIFIER_ONLY_MODE is not enabled'
}

$key3 = [regex]::Match(
    $activeSource,
    '(?s)case KEY3_PRESS:.*?break;')
if (-not $key3.Success) {
    throw 'rectifier-only policy: KEY3 handler is missing'
}
if ($key3.Value -match 'inverter_enable\s*=\s*1') {
    throw 'rectifier-only policy: KEY3 still requests inverter startup'
}
if ($key3.Value -notmatch 'rectifier_enable\s*=\s*1') {
    throw 'rectifier-only policy: KEY3 no longer starts the rectifier'
}

$key6 = [regex]::Match(
    $activeSource,
    '(?s)case KEY6_PRESS:.*?break;')
if (-not $key6.Success) {
    throw 'rectifier-only policy: KEY6 handler is missing'
}
if ($key6.Value -match 'rectifier_enable\s*=|PWM_TripRectifier') {
    throw 'rectifier-only policy: KEY6 still changes rectifier operation'
}

$rectifierStart = [regex]::Match(
    $activeSource,
    '(?s)if\(\(rectifier_enable != 0\).*?rectifier_pwm_start_stage = 1;')
if (-not $rectifierStart.Success) {
    throw 'rectifier-only policy: rectifier startup path is missing'
}
if ($rectifierStart.Value -match 'inverter_pwm_start_stage\s*==\s*3') {
    throw 'rectifier-only policy: rectifier startup still waits for inverter PWM'
}

if ($source -notmatch '(?s)#if\s+!RECTIFIER_ONLY_MODE.*?PWM_ReleaseInverter\(\);.*?#else.*?PWM_TripInverter\(\);.*?#endif') {
    throw 'rectifier-only policy: inverter control is not compile-time disabled and tripped'
}

'controller rectifier-only policy: PASS'
