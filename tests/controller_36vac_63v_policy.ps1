$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\controller.c')

foreach ($required in @(
    '#define INPUT_VOLTAGE_MAX_RMS 41.0f',
    '#define BUS_REF_HEADROOM_VOLTAGE 5.0f',
    '#define BUS_REF_MAX_VOLTAGE 63.0f',
    '#define BUS_OVERVOLTAGE_LIMIT 70.0f',
    'float U_REF = 36.0f;',
    'volatile float U_BUS_REF = 63.0f;',
    'if(U_BUS_REF > BUS_REF_MAX_VOLTAGE) U_BUS_REF = BUS_REF_MAX_VOLTAGE;',
    '1.4142f * INPUT_VOLTAGE_MAX_RMS + BUS_REF_HEADROOM_VOLTAGE'
)) {
    if (-not $source.Contains($required)) { throw "36Vac/63V policy missing: $required" }
}

$requiredBus = [Math]::Sqrt(2.0) * 41.0 + 5.0
if (63.0 -lt $requiredBus) {
    throw "63 V bus is below the required $requiredBus V for 41 Vrms input."
}

'controller 36Vac input / 63V bus policy: PASS'
