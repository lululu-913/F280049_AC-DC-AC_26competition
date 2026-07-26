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

$inputRms = Read-Float 'float\s+U_REF\s*=\s*([0-9.]+)f' '24 Vrms input reference'
$busReference = Read-Float 'U_BUS_REF\s*=\s*([0-9.]+)f' '47.5 V bus reference'
$phaseRms = Read-Float 'U_OUT_REF\s*=\s*([0-9.]+)f' '17.3205 Vrms phase reference'
$overvoltage = Read-Float 'BUS_OVERVOLTAGE_LIMIT\s+([0-9.]+)f' '60 V bus overvoltage limit'
$inputCurrentPeakMaximum = Read-Float 'INPUT_CURRENT_PK_MAX\s+([0-9.]+)f' '6 A input-current reference limit'
$inputOvercurrentLimit = Read-Float 'INPUT_OVERCURRENT_LIMIT\s+([0-9.]+)f' '8 A input overcurrent limit'
$initialModulationDivisor = Read-Float 'float\s+M\s*=\s*([0-9.]+)f' 'nominal modulation divisor'
$modulationMinimum = Read-Float 'if\s*\(\s*M\s*<=\s*([0-9.]+)f\s*\)' 'SVPWM modulation divisor lower limit'

if ([Math]::Abs($inputRms - 24.0) -gt 1.0e-4) {
    throw "Input reference is $inputRms Vrms instead of 24 Vrms."
}
if ([Math]::Abs($busReference - 47.5) -gt 1.0e-4) {
    throw "Bus reference is $busReference V instead of 47.5 V."
}
if ([Math]::Abs($phaseRms - (30.0 / [Math]::Sqrt(3.0))) -gt 1.0e-3) {
    throw "Phase reference $phaseRms Vrms does not produce 30 Vrms line voltage."
}
if ([Math]::Abs($overvoltage - 60.0) -gt 1.0e-4) {
    throw 'Bus overvoltage protection must remain at 60 V.'
}
if ([Math]::Abs($inputCurrentPeakMaximum - 6.0) -gt 1.0e-4 -or
    [Math]::Abs($inputOvercurrentLimit - 8.0) -gt 1.0e-4) {
    throw 'The input-current reference limit must be 6 A and overcurrent protection must remain 8 A.'
}
if ($source -notmatch 'if\s*\(\s*U_BUS_REF\s*>\s*55\.0f\s*\)\s*U_BUS_REF\s*=\s*55\.0f') {
    throw 'Adjustable bus reference must stop at 55 V to retain 5 V before the 60 V trip.'
}
if ($source -match 'case\s+KEY[12]_PRESS:[\s\S]*?U_BUS_REF') {
    throw 'KEY1 and KEY2 must no longer adjust the fixed-default bus reference.'
}

$rectifierMinimumBus = [Math]::Sqrt(2.0) * $inputRms + 5.0
$requiredModulationDivisor = $busReference / ([Math]::Sqrt(2.0) * $phaseRms)

if ($busReference -le $rectifierMinimumBus) {
    throw 'The 47.5 V bus lacks boost-rectifier voltage headroom.'
}
if ($requiredModulationDivisor -le $modulationMinimum) {
    throw 'The nominal 47.5 V bus leaves no SVPWM modulation-divisor margin.'
}
if ([Math]::Abs($initialModulationDivisor - $requiredModulationDivisor) -gt 0.02) {
    throw "Initial M=$initialModulationDivisor does not match nominal M=$requiredModulationDivisor."
}

Write-Output (
    'PASS: 24 Vrms input, 30 Vrms line output, 47.5 V bus, 60 V protection; nominal M={0:F3}.' -f
    $requiredModulationDivisor
)
