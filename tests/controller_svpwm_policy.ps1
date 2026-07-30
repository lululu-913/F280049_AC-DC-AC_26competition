$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

function Assert-Contains {
    param(
        [string]$Pattern,
        [string]$Message
    )

    if ($source -notmatch $Pattern) {
        throw $Message
    }
}

Assert-Contains 'static\s+void\s+SVPWM_Calculate\s*\(' `
    'Missing isolated SVPWM_Calculate function.'
Assert-Contains 'sv_offset\s*=\s*-0\.5f\s*\*\s*\(\s*sv_max\s*\+\s*sv_min\s*\)' `
    'SVPWM common-mode offset must be -(max + min) / 2.'
Assert-Contains '\*duty_a\s*=\s*0\.5f\s*\+\s*ref_a\s*\+\s*sv_offset' `
    'A-phase duty does not include the SVPWM common-mode offset.'
Assert-Contains '\*duty_b\s*=\s*0\.5f\s*\+\s*ref_b\s*\+\s*sv_offset' `
    'B-phase duty does not include the SVPWM common-mode offset.'
Assert-Contains '\*duty_c\s*=\s*0\.5f\s*\+\s*ref_c\s*\+\s*sv_offset' `
    'C-phase duty does not include the SVPWM common-mode offset.'
Assert-Contains 'SVPWM_Calculate\s*\(\s*theta_a\s*,\s*theta_b\s*,\s*theta_c\s*,\s*modulation_a\s*,\s*modulation_b\s*,\s*modulation_c' `
    'The inverter ISR does not call SVPWM with the existing phases and per-phase modulation scales.'
Assert-Contains 'theta_b\s*=\s*theta_a\s*\+\s*2\.0944f' `
    'The existing B-leading phase sequence was not preserved.'
Assert-Contains 'theta_c\s*=\s*theta_a\s*-\s*2\.0944f' `
    'The existing C-lagging phase sequence was not preserved.'
Assert-Contains 'if\s*\(\s*M\s*<=\s*INVERTER_MODULATION_DIVISOR_MIN\s*\)\s*M\s*=\s*INVERTER_MODULATION_DIVISOR_MIN' `
    'The SVPWM modulation divisor lower limit must use its tuning macro.'

if ($source -match 'CMPA\s*=\s*EPWM_TIMER_TBPRD\s*\*\s*inverter_soft_gain\s*\*\s*sinf') {
    throw 'Legacy direct sinusoidal CMPA assignment is still active.'
}

$maximumModulation = 1.0 / 1.8
$tolerance = 1.0e-6

for ($sample = 0; $sample -lt 3600; $sample++) {
    $thetaA = 2.0 * [Math]::PI * $sample / 3600.0
    $thetaB = $thetaA + 2.0944
    $thetaC = $thetaA - 2.0944
    $refA = $maximumModulation * [Math]::Sin($thetaA)
    $refB = $maximumModulation * [Math]::Sin($thetaB)
    $refC = $maximumModulation * [Math]::Sin($thetaC)
    $svMax = [Math]::Max($refA, [Math]::Max($refB, $refC))
    $svMin = [Math]::Min($refA, [Math]::Min($refB, $refC))
    $offset = -0.5 * ($svMax + $svMin)
    $dutyA = 0.5 + $refA + $offset
    $dutyB = 0.5 + $refB + $offset
    $dutyC = 0.5 + $refC + $offset

    if (($dutyA -lt 0.0) -or ($dutyA -gt 1.0) -or
        ($dutyB -lt 0.0) -or ($dutyB -gt 1.0) -or
        ($dutyC -lt 0.0) -or ($dutyC -gt 1.0)) {
        throw "SVPWM duty left the linear range at sample $sample."
    }

    if ([Math]::Abs(($dutyA - $dutyB) - ($refA - $refB)) -gt $tolerance -or
        [Math]::Abs(($dutyB - $dutyC) - ($refB - $refC)) -gt $tolerance) {
        throw "Common-mode injection changed a line-voltage reference at sample $sample."
    }
}

Write-Output 'PASS: SVPWM policy, phase sequence, duty range, and line-voltage invariance.'
