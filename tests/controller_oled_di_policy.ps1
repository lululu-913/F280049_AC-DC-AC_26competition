$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

if ($source -notmatch 'OLED_ShowString\s*\(\s*0\s*,\s*3\s*,\s*"F       DI      "\s*\)') {
    throw 'OLED row 4 must use the exact 16-character F/DI layout.'
}

if ($source -notmatch 'OLED_ShowNum\s*\(\s*1\s*,\s*3\s*,\s*output_freq_hz\s*,\s*4\s*\)') {
    throw 'OLED row 4 must show the adjustable inverter frequency.'
}

if ($source -notmatch 'volatile\s+float\s+Di\s*=\s*0') {
    throw 'ISR-written Di must be volatile when shared with the OLED background task.'
}

if ($source -notmatch 'oled_di_value\s*=\s*Di') {
    throw 'OLED rendering must snapshot Di once before displaying it.'
}

if ($source -notmatch "OLED_ShowChar\s*\(\s*10\s*,\s*3\s*,\s*\(\s*oled_di_value\s*<\s*0\.0f\s*\)\s*\?\s*'-'\s*:\s*'\+'\s*\)") {
    throw 'OLED row 4 must show the sign of its Di snapshot explicitly.'
}

if ($source -notmatch 'OLED_ShowFloat\s*\(\s*11\s*,\s*3\s*,\s*fabsf\s*\(\s*oled_di_value\s*\)\s*,\s*3\s*\)') {
    throw 'OLED row 4 must show the magnitude of its Di snapshot after its sign.'
}

if ($source -match 'OLED_ShowFloat\s*\(\s*10\s*,\s*3\s*,\s*D1\s*,') {
    throw 'OLED row 4 must no longer show D1.'
}

if ($source -notmatch '#define\s+RECTIFIER_MODULATION_LIMIT\s+0\.95f') {
    throw 'The displayed Di range must remain limited to -0.95 through +0.95.'
}
if ($source -notmatch 'if\s*\(\s*Di\s*>\s*RECTIFIER_MODULATION_LIMIT\s*\)\s*Di\s*=\s*RECTIFIER_MODULATION_LIMIT') {
    throw 'Di must retain its positive 0.95 clamp.'
}
if ($source -notmatch 'else\s+if\s*\(\s*Di\s*<\s*-RECTIFIER_MODULATION_LIMIT\s*\)\s*Di\s*=\s*-RECTIFIER_MODULATION_LIMIT') {
    throw 'Di must retain its negative -0.95 clamp.'
}

Write-Output 'PASS: OLED row 4 displays output frequency and signed Di.'
