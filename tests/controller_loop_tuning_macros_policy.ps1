$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\controller.c')

foreach ($required in @(
    '#define RECTIFIER_VOLTAGE_KP 0.075f',
    '#define RECTIFIER_VOLTAGE_KI 0.003f',
    '#define RECTIFIER_CURRENT_KP 10.0f',
    '#define RECTIFIER_CURRENT_KI 0.02f',
    '#define INVERTER_VOLTAGE_KP 0.025f',
    '#define INVERTER_VOLTAGE_KI 0.010f',
    '#define INVERTER_VOLTAGE_ERROR_DEADBAND 0.01f',
    '#define INVERTER_MODULATION_DIVISOR_INITIAL 2.1044f',
    '#define INVERTER_MODULATION_DIVISOR_MIN 1.8f',
    '#define INVERTER_MODULATION_DIVISOR_MAX 8.0f',
    'pid1.Kp = RECTIFIER_VOLTAGE_KP;',
    'pid1.Ki = RECTIFIER_VOLTAGE_KI;',
    'pid2.Kp = RECTIFIER_CURRENT_KP;',
    'pid2.Ki = RECTIFIER_CURRENT_KI;',
    'pida.Kp = INVERTER_VOLTAGE_KP;',
    'pida.Ki = INVERTER_VOLTAGE_KI;',
    'float M = INVERTER_MODULATION_DIVISOR_INITIAL;',
    'if(fabsf(pida.Err) > INVERTER_VOLTAGE_ERROR_DEADBAND)',
    'if(M >= INVERTER_MODULATION_DIVISOR_MAX) M = INVERTER_MODULATION_DIVISOR_MAX;',
    'if(M <= INVERTER_MODULATION_DIVISOR_MIN) M = INVERTER_MODULATION_DIVISOR_MIN;'
)) {
    if (-not $source.Contains($required)) {
        throw "Loop tuning macro policy missing: $required"
    }
}

foreach ($forbidden in @(
    'pid1\.Kp\s*=\s*[0-9]',
    'pid1\.Ki\s*=\s*[0-9]',
    'pid2\.Kp\s*=\s*[0-9]',
    'pid2\.Ki\s*=\s*[0-9]',
    'pida\.Kp\s*=\s*[0-9]',
    'pida\.Ki\s*=\s*[0-9]',
    'float\s+M\s*=\s*[0-9]'
)) {
    if ($source -match $forbidden) {
        throw "A loop gain is still hard-coded outside the macro definitions: $forbidden"
    }
}

'controller loop tuning macro policy: PASS'