$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    'float U_REF = 36.0f;',
    'volatile float U_BUS_REF = 55.0f;',
    '#define BUS_REF_HEADROOM_VOLTAGE 4.0f',
    '#define BUS_REF_MAX_VOLTAGE 65.0f',
    '#define BUS_OVERVOLTAGE_LIMIT 70.0f',
    'if(U_BUS_REF > BUS_REF_MAX_VOLTAGE) U_BUS_REF = BUS_REF_MAX_VOLTAGE;'
)) {
    if (-not $source.Contains($required)) {
        throw "36Vac/55V policy missing: $required"
    }
}

$inputPeak = [Math]::Sqrt(2.0) * 36.0
$headroom = 55.0 - $inputPeak
if ($headroom -lt 4.0) {
    throw '36Vac/55V policy: configured bus target lacks the required 4V boost headroom'
}

$clampOrder = [regex]::Match(
    $source,
    '(?s)if\(U_BUS_REF < \(1\.4142f \* U_REF \+ BUS_REF_HEADROOM_VOLTAGE\)\).*?' +
    'U_BUS_REF = 1\.4142f \* U_REF \+ BUS_REF_HEADROOM_VOLTAGE;.*?' +
    'if\(U_BUS_REF > BUS_REF_MAX_VOLTAGE\) U_BUS_REF = BUS_REF_MAX_VOLTAGE;')
if (-not $clampOrder.Success) {
    throw '36Vac/55V policy: final bus-reference maximum clamp must run after the boost-headroom clamp'
}

'controller 36Vac input / 55V bus policy: PASS'
