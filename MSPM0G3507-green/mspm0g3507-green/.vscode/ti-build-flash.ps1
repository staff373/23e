[CmdletBinding()]
param(
    [switch]$Flash
)

$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildDir = Join-Path $projectRoot "Debug"
$projectName = Split-Path $projectRoot -Leaf

$ccsRoot = "E:\ti\ccs2050\ccs"
$sdkRoot = "C:\ti\mspm0_sdk_2_10_00_04"
$compilerRoot = Join-Path $ccsRoot "tools\compiler\ti-cgt-armllvm_4.0.4.LTS"
$compiler = Join-Path $compilerRoot "bin\tiarmclang.exe"
$sysconfig = "C:\ti\sysconfig_1.26.2\sysconfig_cli.bat"
$dslite = Join-Path $ccsRoot "ccs_base\DebugServer\bin\DSLite.exe"
$ccxml = Join-Path $projectRoot "targetConfigs\MSPM0G3507.ccxml"

foreach ($path in @($compiler, $sysconfig, $sdkRoot)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required TI path not found: $path"
    }
}

New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$syscfgFiles = @(Get-ChildItem -LiteralPath $projectRoot -Filter "*.syscfg" -File)
if ($syscfgFiles.Count -ne 1) {
    throw "Expected exactly one .syscfg file in project root, found $($syscfgFiles.Count)."
}

Write-Host "==> Generating SysConfig"
& $sysconfig `
    --product (Join-Path $sdkRoot ".metadata\product.json") `
    --script $syscfgFiles[0].FullName `
    --output $buildDir `
    --compiler ticlang
if ($LASTEXITCODE -ne 0) {
    throw "SysConfig generation failed with exit code $LASTEXITCODE."
}

$deviceOpt = Join-Path $buildDir "device.opt"
$commonCompileArgs = @(
    "-c",
    "-O2",
    "-gdwarf-3",
    "-mcpu=cortex-m0plus",
    "-march=thumbv6m",
    "-mfloat-abi=soft",
    "-mthumb",
    "-Wall",
    "-I$projectRoot",
    "-I$buildDir",
    "-I$(Join-Path $projectRoot "modular\app_green_indicator")",
    "-I$(Join-Path $projectRoot "modular\app_keys")",
    "-I$(Join-Path $projectRoot "modular\app_vision_comm")",
    "-I$(Join-Path $projectRoot "modular\bsp_bt")",
    "-I$(Join-Path $projectRoot "modular\bsp_jy61p")",
    "-I$(Join-Path $projectRoot "modular\bsp_key_leds")",
    "-I$(Join-Path $projectRoot "modular\bsp_keys")",
    "-I$(Join-Path $projectRoot "modular\bsp_motor")",
    "-I$(Join-Path $projectRoot "modular\bsp_oled")",
    "-I$(Join-Path $projectRoot "modular\bsp_stepper")",
    "-I$(Join-Path $projectRoot "modular\bsp_vision_uart")",
    "-I$(Join-Path $sdkRoot "source\third_party\CMSIS\Core\Include")",
    "-I$(Join-Path $sdkRoot "source")",
    "@$deviceOpt"
)

$startupSource = Join-Path $sdkRoot "source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c"
$sourceFiles = @()
$sourceFiles += @(Get-ChildItem -LiteralPath $projectRoot -Filter "*.c" -File | ForEach-Object { $_.FullName })
$modularDir = Join-Path $projectRoot "modular"
if (Test-Path -LiteralPath $modularDir) {
    $sourceFiles += @(Get-ChildItem -LiteralPath $modularDir -Filter "*.c" -Recurse -File | ForEach-Object { $_.FullName })
}
$sourceFiles += Join-Path $buildDir "ti_msp_dl_config.c"
$sourceFiles += $startupSource

$objects = @()
foreach ($source in $sourceFiles) {
    $baseName = [System.IO.Path]::GetFileNameWithoutExtension($source)
    $relativeSource = $source
    if ($source.StartsWith($projectRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        $relativeSource = $source.Substring($projectRoot.Length).TrimStart('\', '/')
    }
    $objectName = ($relativeSource -replace '[\\/:\.]', '_') + ".obj"
    $object = Join-Path $buildDir $objectName
    $objects += $object

    Write-Host "==> Compiling $relativeSource"
    & $compiler @commonCompileArgs $source -o $object
    if ($LASTEXITCODE -ne 0) {
        throw "Compile failed for $source with exit code $LASTEXITCODE."
    }
}

$outFile = Join-Path $buildDir "$projectName.out"
$mapFile = Join-Path $buildDir "$projectName.map"
$linkArgs = @(
    "-mcpu=cortex-m0plus",
    "-march=thumbv6m",
    "-mfloat-abi=soft",
    "-mthumb",
    "-Wl,--rom_model",
    "-Wl,--warn_sections",
    "-Wl,-m$mapFile",
    "-L$(Join-Path $sdkRoot "source")",
    "-L$projectRoot",
    "-L$buildDir",
    "-L$(Join-Path $compilerRoot "lib")"
)
$linkArgs += $objects
$linkArgs += Join-Path $buildDir "device_linker.cmd"
$linkArgs += "-Wl,--stack_size=4096"
$linkArgs += Join-Path $buildDir "device.cmd.genlibs"
$linkArgs += Join-Path $compilerRoot "lib\libc.a"
$linkArgs += @("-o", $outFile)

Write-Host "==> Linking $outFile"
& $compiler @linkArgs
if ($LASTEXITCODE -ne 0) {
    throw "Link failed with exit code $LASTEXITCODE."
}

Write-Host "Build OK: $outFile"

if ($Flash) {
    if (-not (Test-Path -LiteralPath $dslite)) {
        throw "DSLite not found: $dslite"
    }
    if (-not (Test-Path -LiteralPath $ccxml)) {
        throw "Target configuration not found: $ccxml"
    }

    $flashLog = Join-Path $buildDir "dslite.log"
    Write-Host "==> Flashing with XDS110"
    & $dslite load --config $ccxml --file $outFile --timeout 60 --log $flashLog
    if ($LASTEXITCODE -ne 0) {
        throw "Flash failed with exit code $LASTEXITCODE. See log: $flashLog"
    }

    Write-Host "Flash OK, target is running."
}
