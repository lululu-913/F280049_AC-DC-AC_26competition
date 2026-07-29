$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

function Read-Float {
    param(
        [string]$Pattern,
        [string]$Description
    )

    $match = [regex]::Match($source, $Pattern)
    if (-not $match.Success) {
        throw "Missing $Description."
    }

    return [double]::Parse(
        $match.Groups[1].Value,
        [Globalization.CultureInfo]::InvariantCulture
    )
}

$inputRms = Read-Float 'float\s+U_REF\s*=\s*([0-9.]+)f' '36 Vrms input reference'
$busReference = Read-Float 'U_BUS_REF\s*=\s*([0-9.]+)f' '55 V bus reference'
$phaseRms = Read-Float 'U_OUT_REF\s*=\s*([0-9.]+)f' '17.3205 Vrms phase reference'
$overvoltage = Read-Float 'BUS_OVERVOLTAGE_LIMIT\s+([0-9.]+)f' '70 V bus overvoltage limit'
$inputCurrentPeakMaximum = Read-Float 'INPUT_CURRENT_PK_MAX\s+([0-9.]+)f' '6 A input-current reference limit'
$inputOvercurrentLimit = Read-Float 'INPUT_OVERCURRENT_LIMIT\s+([0-9.]+)f' '9 A input overcurrent limit'

if ([Math]::Abs($inputRms - 36.0) -gt 1.0e-4) {
    throw "Input reference is $inputRms Vrms instead of 36 Vrms."
}
if ([Math]::Abs($busReference - 55.0) -gt 1.0e-4) {
    throw "Bus reference is $busReference V instead of 55 V."
}
if ([Math]::Abs($phaseRms - (30.0 / [Math]::Sqrt(3.0))) -gt 1.0e-3) {
    throw "Phase reference $phaseRms Vrms does not produce 30 Vrms line voltage."
}
if ([Math]::Abs($overvoltage - 70.0) -gt 1.0e-4) {
    throw 'Bus overvoltage protection must remain at 70 V.'
}
if ([Math]::Abs($inputCurrentPeakMaximum - 6.0) -gt 1.0e-4 -or
    [Math]::Abs($inputOvercurrentLimit - 9.0) -gt 1.0e-4) {
    throw 'The input-current reference limit must be 6 A and overcurrent protection must remain 9 A.'
}
if ($source -notmatch '(?s)U_BUS_REF\s*=\s*1\.4142f\s*\*\s*U_REF\s*\+\s*BUS_REF_HEADROOM_VOLTAGE;.*?if\s*\(\s*U_BUS_REF\s*>\s*BUS_REF_MAX_VOLTAGE\s*\)\s*U_BUS_REF\s*=\s*BUS_REF_MAX_VOLTAGE') {
    throw 'The final bus-reference clamp must stop at the configured maximum after applying boost headroom.'
}
foreach ($keyName in @('KEY1_PRESS', 'KEY2_PRESS')) {
    $keyCase = [regex]::Match($source, "(?s)case\s+${keyName}:.*?break;")
    if (-not $keyCase.Success -or $keyCase.Value -match 'U_BUS_REF') {
        throw "$keyName must not adjust the bus reference."
    }
}

$rectifierMinimumBus = [Math]::Sqrt(2.0) * $inputRms + 4.0

if ($busReference -le $rectifierMinimumBus) {
    throw 'The 55 V bus lacks the required 4 V boost-rectifier headroom.'
}

Write-Output (
    'PASS: 36 Vrms input, 55 V rectifier bus, 70 V protection, 4 V boost headroom.'
)
