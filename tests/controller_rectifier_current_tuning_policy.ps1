$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    '#define RECTIFIER_CURRENT_KP 10.0f',
    '#define RECTIFIER_CURRENT_KI 0.02f',
    '#define RECTIFIER_FEEDFORWARD_GAIN 1.00f',
    'pid2.Kp = RECTIFIER_CURRENT_KP;',
    'pid2.Ki = RECTIFIER_CURRENT_KI;',
    'i_ctrl = u_i_out + RECTIFIER_FEEDFORWARD_GAIN * U_in;'
)) {
    if (-not $source.Contains($required)) {
        throw "rectifier current tuning policy missing: $required"
    }
}

'controller rectifier current tuning policy: PASS'
