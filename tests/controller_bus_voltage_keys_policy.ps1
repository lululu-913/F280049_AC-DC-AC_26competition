$ErrorActionPreference = 'Stop'

$path = Join-Path $PSScriptRoot '..\controller.c'
$source = Get-Content -Raw -LiteralPath $path

foreach ($required in @(
    '#define BUS_REF_MIN_VOLTAGE 55.0f',
    '#define BUS_REF_MAX_VOLTAGE 65.0f',
    '#define BUS_REF_STEP_VOLTAGE 0.5f',
    'if(U_BUS_REF > BUS_REF_MAX_VOLTAGE) U_BUS_REF = BUS_REF_MAX_VOLTAGE;'
)) {
    if (-not $source.Contains($required)) {
        throw "bus voltage key policy missing: $required"
    }
}

$key4 = [regex]::Match($source, '(?s)case KEY4_PRESS:.*?break;')
if (-not $key4.Success -or
    $key4.Value -notmatch 'U_BUS_REF\s*\+=\s*BUS_REF_STEP_VOLTAGE' -or
    $key4.Value -notmatch 'BUS_REF_MAX_VOLTAGE' -or
    $key4.Value -match 'U_OUT_REF') {
    throw 'bus voltage key policy: KEY4 must only increase the bounded bus reference'
}

$key5 = [regex]::Match($source, '(?s)case KEY5_PRESS:.*?break;')
if (-not $key5.Success -or
    $key5.Value -notmatch 'U_BUS_REF\s*-=\s*BUS_REF_STEP_VOLTAGE' -or
    $key5.Value -notmatch 'BUS_REF_MIN_VOLTAGE' -or
    $key5.Value -match 'U_OUT_REF') {
    throw 'bus voltage key policy: KEY5 must only decrease the bounded bus reference'
}

'controller KEY4/KEY5 bus voltage policy: PASS'
