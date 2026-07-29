$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    '#define BOOTSTRAP_CHARGE_TEST_MODE 0',
    '#if BOOTSTRAP_CHARGE_TEST_MODE',
    'static void PWM_ForceRectifierLowSidesOn(void);',
    'static void PWM_ReleaseRectifier(void);'
)) {
    if (-not $source.Contains($required)) {
        throw "bootstrap charge test policy missing: $required"
    }
}

if ($source -notmatch '(?s)#if\s+!BOOTSTRAP_CHARGE_TEST_MODE.*?if\(rectifier_enable != rectifier_enable_last\).*?PWM_ReleaseRectifier\(\);.*?#endif') {
    throw 'bootstrap restore policy: normal rectifier state machine is not restored'
}

if ($source -notmatch '(?s)#if\s+!BOOTSTRAP_CHARGE_TEST_MODE.*?N_freq_key\+\+;.*?KEY_Control\(key\);.*?#endif') {
    throw 'bootstrap restore policy: normal key processing is not restored'
}

'controller normal rectifier PWM restored: PASS'
