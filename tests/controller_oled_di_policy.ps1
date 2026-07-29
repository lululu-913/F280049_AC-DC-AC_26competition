$ErrorActionPreference = 'Stop'

$controllerPath = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $controllerPath

if ($source -notmatch 'OLED_ShowString\s*\(\s*0\s*,\s*3\s*,\s*"P               "\s*\)') {
    throw 'OLED row 4 must use the exact 16-character P layout.'
}

if ($source -notmatch 'oled_pid_out_value\s*=\s*pid_out') {
    throw 'OLED rendering must snapshot pid_out before displaying it.'
}

if ($source -notmatch 'volatile\s+float\s+pid_out\s*=\s*0') {
    throw 'pid_out must be volatile because the ISR writes it and the foreground OLED reads it.'
}

if ($source -notmatch 'OLED_ShowFloat\s*\(\s*2\s*,\s*3\s*,\s*oled_pid_out_value\s*,\s*3\s*\)') {
    throw 'OLED row 4 must display the pid_out snapshot after P.'
}

$oledFunction = [regex]::Match(
    $source,
    '(?s)void\s+OLED_output\s*\(\s*void\s*\)[^\r\n]*\r?\n\s*\{.*?(?=void\s+KEY_Control\s*\(\s*int\s+key_value\s*\)[^\r\n]*\r?\n\s*\{)')
if (-not $oledFunction.Success) {
    throw 'OLED_output function is missing.'
}

$row4 = [regex]::Match(
    $oledFunction.Value,
    '(?s)default:.*?break;')
if (-not $row4.Success) {
    throw 'OLED row 4 rendering branch is missing.'
}
if ($oledFunction.Value -match 'output_freq_hz|oled_di_value|\bDi\b|OLED_ShowChar\s*\([^,]+,\s*3\s*,') {
    throw 'OLED row 4 still contains the old frequency or Di display.'
}

Write-Output 'PASS: OLED row 4 displays pid_out as P.'
