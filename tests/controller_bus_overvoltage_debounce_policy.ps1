$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

if ($source -notmatch '#define\s+BUS_OVERVOLTAGE_LIMIT\s+70\.0f') {
    throw 'Bus overvoltage threshold must remain 70.0V.'
}

if ($source -notmatch '#define\s+BUS_OVERVOLTAGE_CONFIRM_SAMPLES\s+40U') {
    throw 'Bus overvoltage must require exactly 40 consecutive 20kHz samples (2ms).'
}

if ($source -notmatch 'Uint16\s+bus_overvoltage_count\s*=\s*0U') {
    throw 'Bus overvoltage consecutive-sample counter is missing.'
}

$isr = [regex]::Match(
    $source,
    '(?s)__interrupt\s+void\s+adcA1ISR\s*\(\s*void\s*\)[^\r\n]*\r?\n\s*\{.*?(?=\r?\n\})')
if (-not $isr.Success) {
    throw 'adcA1ISR definition is missing.'
}
$isrCode = [regex]::Replace($isr.Value, '//[^\r\n]*', '')

$confirmationBlock = [regex]::Match(
    $isrCode,
    '(?s)if\s*\(\s*U_bus\s*>=\s*BUS_OVERVOLTAGE_LIMIT\s*\)\s*' +
    '\{\s*if\s*\(\s*bus_overvoltage_count\s*<\s*BUS_OVERVOLTAGE_CONFIRM_SAMPLES\s*\)\s*' +
    'bus_overvoltage_count\s*\+\+\s*;\s*\}\s*else\s*' +
    '\{\s*bus_overvoltage_count\s*=\s*0U\s*;\s*\}\s*' +
    'if\s*\(\s*bus_overvoltage_count\s*>=\s*BUS_OVERVOLTAGE_CONFIRM_SAMPLES\s*\)\s*' +
    '\{.*?PWM_TripRectifier\s*\(\s*\)\s*;.*?flag\s*=\s*2\s*;\s*\}')
if (-not $confirmationBlock.Success) {
    throw 'F2 must use saturating consecutive counting, reset below 70V, and trip only after count 40.'
}

$protectionRegion = [regex]::Match(
    $isrCode,
    '(?s)I_in\s*=.*?(?=if\s*\(\s*system_fault\s*!=\s*0\s*\))')
if (-not $protectionRegion.Success) {
    throw 'ISR protection region is missing.'
}
if ([regex]::Matches($protectionRegion.Value, 'flag\s*=\s*2\s*;').Count -ne 1) {
    throw 'F2 must have exactly one assignment in the ISR protection region.'
}
if ([regex]::Matches($protectionRegion.Value, 'U_bus\s*>=\s*BUS_OVERVOLTAGE_LIMIT').Count -ne 1) {
    throw 'A parallel single-sample overvoltage path is not allowed.'
}

Write-Output 'PASS: F2 requires 2ms of consecutive samples at or above 70V.'
