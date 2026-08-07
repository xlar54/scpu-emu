$ErrorActionPreference = "Stop"

$testRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$binDir = Join-Path $testRoot "bin"
$objDir = Join-Path $testRoot "obj"
$petcat = Join-Path $testRoot "petcat.exe"
$c1541 = Join-Path $testRoot "c1541.exe"

if (-not (Test-Path $petcat)) { $petcat = "petcat" }
if (-not (Test-Path $c1541)) { $c1541 = "c1541" }

New-Item -ItemType Directory -Force -Path $binDir, $objDir | Out-Null

foreach ($source in Get-ChildItem -Path (Join-Path $testRoot "basic") -Filter "*.bas") {
    $output = Join-Path $binDir ($source.BaseName.ToUpperInvariant() + ".prg")
    & $petcat -w2 -o $output -- $source.FullName
    if ($LASTEXITCODE -ne 0) { throw "petcat failed for $($source.Name)" }
}

foreach ($source in Get-ChildItem -Path (Join-Path $testRoot "ml") -Filter "*.asm") {
    $object = Join-Path $objDir ($source.BaseName + ".o")
    $output = Join-Path $binDir ($source.BaseName.ToUpperInvariant() + ".prg")
    $linkConfig = if ($source.Name -eq "15-scpu128probe.asm") {
        Join-Path $testRoot "ml\c128-prg.cfg"
    } else {
        Join-Path $testRoot "ml\c64-prg.cfg"
    }
    & ca65 --cpu 65816 -I (Join-Path $testRoot "ml") -o $object $source.FullName
    if ($LASTEXITCODE -ne 0) { throw "ca65 failed for $($source.Name)" }
    & ld65 -C $linkConfig -o $output $object
    if ($LASTEXITCODE -ne 0) { throw "ld65 failed for $($source.Name)" }
}

$diskImage = Join-Path $testRoot "SCPU-TESTS.d64"
$diskArgs = @("-format", "scpu tests,st", "d64", $diskImage)
foreach ($program in Get-ChildItem -Path $binDir -Filter "*.prg" | Sort-Object Name) {
    $diskArgs += @("-write", $program.FullName, $program.BaseName)
}
& $c1541 @diskArgs
if ($LASTEXITCODE -ne 0) { throw "c1541 failed to build $diskImage" }

Write-Host "Built PRG files in $binDir"
Write-Host "Built disk image $diskImage"
