$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

if ($source -notmatch 'OLED_ShowString\s*\(\s*0\s*,\s*3\s*,\s*"IR      DI      "\s*\)') {
    throw 'OLED row 4 must use the exact 16-character IR/DI layout.'
}

if ($source -notmatch "OLED_ShowChar\s*\(\s*10\s*,\s*3\s*,\s*\(\s*Di\s*<\s*0\.0f\s*\)\s*\?\s*'-'\s*:\s*'\+'\s*\)") {
    throw 'OLED row 4 must show the sign of Di explicitly.'
}

if ($source -notmatch 'OLED_ShowFloat\s*\(\s*11\s*,\s*3\s*,\s*fabsf\s*\(\s*Di\s*\)\s*,\s*3\s*\)') {
    throw 'OLED row 4 must show the magnitude of Di after its sign.'
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

Write-Output 'PASS: OLED row 4 displays signed Di in the -0.95 to +0.95 range.'
