$ErrorActionPreference = 'Stop'

$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\controller.c')

function Read-Float([string]$pattern, [string]$description) {
    $match = [regex]::Match($source, $pattern)
    if (-not $match.Success) { throw "Missing $description." }
    return [double]::Parse(
        $match.Groups[1].Value,
        [Globalization.CultureInfo]::InvariantCulture)
}

$phaseReference = Read-Float `
    'U_OUT_REF\s*=\s*([0-9.]+)f' `
    'requirement-1 phase-voltage reference'
$busReference = Read-Float `
    'U_BUS_REF\s*=\s*([0-9.]+)f' `
    'nominal DC-bus reference'
$initialM = Read-Float `
    'INVERTER_MODULATION_DIVISOR_INITIAL\s+([0-9.]+)f' `
    'nominal inverter modulation denominator'
$maximumPhaseReference = Read-Float `
    'OUTPUT_PHASE_REF_MAX_RMS\s+([0-9.]+)f' `
    'phase-reference clamp'

$expectedPhase = 32.0 / [Math]::Sqrt(3.0)
$lineFromReference = $phaseReference * [Math]::Sqrt(3.0)
$lineFromInitialM = $busReference * [Math]::Sqrt(3.0 / 2.0) / $initialM

if ([Math]::Abs($phaseReference - $expectedPhase) -gt 1.0e-3) {
    throw "Phase reference $phaseReference Vrms does not represent a 32 Vrms line voltage."
}
if ([Math]::Abs($lineFromReference - 32.0) -gt 0.01) {
    throw "Configured phase reference predicts $lineFromReference Vrms line voltage."
}
if ([Math]::Abs($lineFromInitialM - 32.0) -gt 0.02) {
    throw "Initial M predicts $lineFromInitialM Vrms line voltage at the nominal bus."
}
if ($maximumPhaseReference -lt $phaseReference) {
    throw 'The phase-reference clamp would reduce the 32 Vrms line target.'
}

foreach ($required in @(
    '#define OUTPUT_FREQ_HIGH_HZ 60.0f',
    '#define INVERTER_EPWM_TBPRD  2500',
    '#define INVERTER_PWM_RELEASE_WAIT_ISR 2U',
    'Uint16 inverter_pwm_release_wait = 0U',
    'if(inverter_pwm_release_wait < INVERTER_PWM_RELEASE_WAIT_ISR)',
    'inverter_pwm_release_wait++',
    'if(inverter_pwm_release_wait >= INVERTER_PWM_RELEASE_WAIT_ISR)',
    'if(U_OUT_REF > OUTPUT_PHASE_REF_MAX_RMS) U_OUT_REF = OUTPUT_PHASE_REF_MAX_RMS;'
)) {
    if (-not $source.Contains($required)) {
        throw "Requirement-1 inverter policy missing: $required"
    }
}

if ($source.Contains('INVERTER_BUS_START_MIN')) {
    throw 'A bus-voltage startup threshold was added contrary to the approved design.'
}

'PASS: requirement-1 inverter target and safe PWM release policy.'
