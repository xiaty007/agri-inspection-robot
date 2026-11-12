param([switch]$Production)
$ErrorActionPreference = 'Stop'
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$projectFile = @(Get-ChildItem -LiteralPath $workspace -Directory |
    Where-Object { $_.Name -like '407_gps*' } |
    ForEach-Object { Join-Path $_.FullName 'MDK-ARM/112.uvprojx' })
if ($projectFile.Count -ne 1) { throw 'Expected one active STM32 project.' }
[xml]$project = Get-Content -LiteralPath $projectFile[0] -Raw
$projectDir = Split-Path $projectFile[0]
$target = $project.Project.Targets.Target
$toolDir = 'C:/Keil_v5/ARM/ARMCC/bin'
$variant = if ($Production) { 'firmware_normal' } else { 'firmware_fifo_test' }
$outputDir = Join-Path $PSScriptRoot "build/$variant"
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$cArgs = @('--cpu', 'Cortex-M4.fp.sp', '--c99', '-O0', '--split_sections',
    '--library_type=microlib', '-D__MICROLIB', '-c')
if ($Production) { $cArgs += '-DPNI_FIFO_TEST_ENABLE=0' }
foreach ($define in $target.TargetOption.TargetArmAds.Cads.VariousControls.Define.Split(',')) {
    if ($define) { $cArgs += "-D$define" }
}
foreach ($include in $target.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath.Split(';')) {
    $cArgs += @('-I', [IO.Path]::GetFullPath((Join-Path $projectDir $include)))
}
$objects = @()
foreach ($group in $target.Groups.Group) {
    foreach ($file in $group.Files.File) {
        if ($file.FileType -notin '1', '2') { continue }
        if ($file.FileOption.CommonProperty.IncludeInBuild -eq '0') { continue }
        $source = [IO.Path]::GetFullPath((Join-Path $projectDir $file.FilePath))
        $object = Join-Path $outputDir ([IO.Path]::GetFileNameWithoutExtension($source) + '.o')
        if ($file.FileType -eq '1') {
            & (Join-Path $toolDir 'armcc.exe') @cArgs '-o' $object $source
        } else {
            & (Join-Path $toolDir 'armasm.exe') '--cpu' 'Cortex-M4.fp.sp' '--pd' '__MICROLIB SETA 1' '-o' $object $source
        }
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $source" }
        $objects += $object
    }
}
$axf = Join-Path $outputDir '112.axf'
$hex = Join-Path $outputDir '112.hex'
$scatter = Join-Path $projectDir $target.TargetOption.TargetArmAds.LDads.ScatterFile
& (Join-Path $toolDir 'armlink.exe') '--cpu' 'Cortex-M4.fp.sp' '--library_type=microlib' '--scatter' $scatter '--map' '--info=sizes,stack' '--list' (Join-Path $outputDir '112.map') '--output' $axf @objects
if ($LASTEXITCODE -ne 0) { throw 'Link failed.' }
& (Join-Path $toolDir 'fromelf.exe') '--i32combined' '--output' $hex $axf
if ($LASTEXITCODE -ne 0) { throw 'HEX conversion failed.' }
Write-Output "Built $hex"
