$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\controller.c')

foreach ($required in @(
    '#define INPUT_OVERCURRENT_CONFIRM_SAMPLES 3U',
    'Uint16 input_overcurrent_count = 0U',
    'if(input_overcurrent_count < INPUT_OVERCURRENT_CONFIRM_SAMPLES)',
    'input_overcurrent_count++;',
    'input_overcurrent_count = 0U;',
    'if(input_overcurrent_count >= INPUT_OVERCURRENT_CONFIRM_SAMPLES)'
)) {
    if (-not $source.Contains($required)) {
        throw "F1 consecutive-confirmation policy missing: $required"
    }
}

$overcurrentBlock = [regex]::Match(
    $source,
    '(?s)if\(\(system_fault == 0\).*?fabsf\(I_in\) > INPUT_OVERCURRENT_LIMIT\)\).*?flag = 1;.*?\}')

if (-not $overcurrentBlock.Success) {
    throw 'Missing rectifier input-overcurrent protection block.'
}

if ($overcurrentBlock.Value -notmatch 'rectifier_pwm_start_stage\s*==\s*3') {
    throw 'F1 can still latch before active rectifier PWM reaches the normal running stage.'
}

foreach ($required in @(
    'rectifier_enable = 0;',
    'rectifier_fault = 1;',
    'rectifier_pwm_start_stage = 0;',
    'PWM_TripRectifier();',
    'flag = 1;'
)) {
    if (-not $overcurrentBlock.Value.Contains($required)) {
        throw "Running-state overcurrent shutdown action missing: $required"
    }
}

'PASS: F1 is stage-3 armed and requires three consecutive samples (up to 150us).'
