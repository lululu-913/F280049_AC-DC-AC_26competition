$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

foreach ($required in @(
    '#include "adc_average.h"',
    '#define ADC_AVERAGE_WINDOW_SAMPLES 6000U',
    'ADC_AverageState adcb_result0_average = {0UL, 0U, 0U, 0.0f};',
    'av = ADC_Average_Update(&adcb_result0_average, AdcbResultRegs.ADCRESULT0, ADC_AVERAGE_WINDOW_SAMPLES);'
)) {
    if (-not $source.Contains($required)) {
        throw "Reusable ADC average implementation missing: $required"
    }
}

if ($source -match 'su\s*\+=\s*AdcbResultRegs\.ADCRESULT') {
    throw 'Legacy channel-specific ADC averaging must be removed.'
}

$gcc = Get-Command gcc -ErrorAction Stop
$harnessPath = Join-Path $PSScriptRoot 'adc_average_harness.c.txt'
$executablePath = Join-Path ([System.IO.Path]::GetTempPath()) ("adc_average_harness_{0}.exe" -f [guid]::NewGuid())
try {
    & $gcc.Source -x c -std=c99 -Wall -Wextra -Werror $harnessPath -o $executablePath
    if ($LASTEXITCODE -ne 0) {
        throw 'ADC average C behavior harness failed to compile.'
    }
    & $executablePath
    if ($LASTEXITCODE -ne 0) {
        throw "ADC average C behavior harness failed with exit code $LASTEXITCODE."
    }
}
finally {
    if (Test-Path -LiteralPath $executablePath) {
        Remove-Item -LiteralPath $executablePath -Force
    }
}

Write-Output 'PASS: reusable ADC block-average function is configured for ADCB RESULT0.'
