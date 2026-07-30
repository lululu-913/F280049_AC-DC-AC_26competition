$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\controller.c')

function Read-Float([string]$pattern, [string]$description) {
    $match = [regex]::Match($source, $pattern)
    if (-not $match.Success) { throw "Missing $description." }
    return [double]::Parse($match.Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture)
}

$inputRated = Read-Float 'float\s+U_REF\s*=\s*([0-9.]+)f' '36 Vrms rated input'
$inputMaximum = Read-Float 'INPUT_VOLTAGE_MAX_RMS\s+([0-9.]+)f' '41 Vrms maximum input'
$busReference = Read-Float 'U_BUS_REF\s*=\s*([0-9.]+)f' '63 V bus reference'
$busMaximum = Read-Float 'BUS_REF_MAX_VOLTAGE\s+([0-9.]+)f' '63 V bus clamp'
$busHeadroom = Read-Float 'BUS_REF_HEADROOM_VOLTAGE\s+([0-9.]+)f' '5 V boost headroom'
$overvoltage = Read-Float 'BUS_OVERVOLTAGE_LIMIT\s+([0-9.]+)f' '70 V overvoltage protection'
$phaseReference = Read-Float 'U_OUT_REF\s*=\s*([0-9.]+)f' '32 V line-output phase reference'
$modulationDenominator = Read-Float 'float\s+M\s*=\s*([0-9.]+)f' 'nominal inverter modulation denominator'
$busFeedforwardInitial = Read-Float 'float\s+U_bus_ff\s*=\s*([0-9.]+)f' 'bus feedforward initial value'

if ([Math]::Abs($inputRated - 36.0) -gt 1.0e-4) { throw "Rated input is $inputRated Vrms instead of 36 Vrms." }
if ([Math]::Abs($inputMaximum - 41.0) -gt 1.0e-4) { throw "Maximum input is $inputMaximum Vrms instead of 41 Vrms." }
if ([Math]::Abs($busReference - 63.0) -gt 1.0e-4 -or [Math]::Abs($busMaximum - 63.0) -gt 1.0e-4) {
    throw "Bus reference/clamp must both be 63 V, got $busReference/$busMaximum V."
}
if ([Math]::Abs($busHeadroom - 5.0) -gt 1.0e-4) { throw 'Boost headroom must be 5 V.' }
if ($busReference + 1.0e-4 -lt ([Math]::Sqrt(2.0) * $inputMaximum + $busHeadroom)) {
    throw '63 V bus does not cover the 41 Vrms input peak plus 5 V boost headroom.'
}
if ([Math]::Abs($overvoltage - 70.0) -gt 1.0e-4) { throw 'Bus overvoltage protection must be 70 V.' }

$expectedPhase = 32.0 / [Math]::Sqrt(3.0)
if ([Math]::Abs($phaseReference - $expectedPhase) -gt 1.0e-3) {
    throw "Phase reference $phaseReference Vrms does not produce 32 Vrms line voltage."
}

$predictedLine = $busReference * [Math]::Sqrt(3.0 / 2.0) / $modulationDenominator
if ([Math]::Abs($predictedLine - 32.0) -gt 0.02) {
    throw "Initial modulation predicts $predictedLine Vrms line voltage instead of 32 Vrms."
}
if ((1.0 / $modulationDenominator) -gt 0.56) { throw 'Nominal modulation exceeds the configured SVPWM limit.' }
if ([Math]::Abs($busFeedforwardInitial - $busReference) -gt 1.0e-4) {
    throw 'Bus feedforward initial value must match the 63 V bus reference.'
}

foreach ($required in @(
    'if(U_BUS_REF > BUS_REF_MAX_VOLTAGE) U_BUS_REF = BUS_REF_MAX_VOLTAGE;',
    '1.4142f * INPUT_VOLTAGE_MAX_RMS + BUS_REF_HEADROOM_VOLTAGE'
)) {
    if (-not $source.Contains($required)) { throw "Competition voltage policy missing: $required" }
}

'controller competition voltage policy: PASS'
