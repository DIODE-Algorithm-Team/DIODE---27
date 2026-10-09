# 注意：本文件保存为 UTF-8 with BOM。Windows PowerShell 5.1 在无 BOM 时会把中文
# 注释按 GBK 解析，可能导致脚本编译崩溃（0xC0000005）。改动后请保持 BOM。
<#
.SYNOPSIS
    前哨站（Outpost）4 个测试的 Windows 本地运行脚本（MSVC + vcpkg/conda 依赖）。

.DESCRIPTION
    仓库自带 CMakeLists.txt 在 Windows 上无法 configure：
      - CMakeLists.txt L15 把 OpenVINO_DIR 硬编码成 Linux 路径；
      - io / tasks/auto_aim 依赖 SocketCAN、MindVision/HikRobot .so、libusb 等 Linux 专有件。
    本脚本改用等价测试工程 out/win_outpost_tests（源码与仓库测试完全相同，仅替换
    Linux 专有的 io/cboard.hpp 为同枚举替身头文件），编译并运行：
      outpost_helper_test / outpost_geometry_test / outpost_target_test / outpost_aimer_test
    Linux 机器请改用 tests/run_outpost_tests.sh。

.PARAMETER Clean
    先删除 out/win_outpost_tests/build 再重新配置。

.PARAMETER Config
    Visual Studio 生成器的配置，默认 Release。

.PARAMETER ConfigPath
    outpost_aimer_test 使用的 Aimer YAML（相对仓库根目录），默认 configs/sentry.yaml。

.PARAMETER Generator
    CMake 生成器，默认 "Visual Studio 17 2022"。

.EXAMPLE
    pwsh -File tests/run_outpost_tests.ps1
.EXAMPLE
    pwsh -File tests/run_outpost_tests.ps1 -Clean -Config RelWithDebInfo
#>
[CmdletBinding()]
param(
    [switch]$Clean,
    [string]$Config = 'Release',
    [string]$ConfigPath = 'configs/sentry.yaml',
    [string]$Generator = 'Visual Studio 17 2022',
    [string]$VcpkgRoot,
    [string]$CondaRoot
)

$ErrorActionPreference = 'Stop'
# 让原生命令（cmake/g++/测试程序）的 stderr 不因 ErrorActionPreference 中断脚本
if (Get-Variable -Name PSNativeCommandUseErrorActionPreference -Scope Global -ErrorAction SilentlyContinue) {
    $PSNativeCommandUseErrorActionPreference = $false
}

# Windows PowerShell 5.1 会把原生命令的 stderr（如 "CMake Warning"）当作 ErrorRecord，
# 在 $ErrorActionPreference='Stop' 下直接抛 NativeCommandError。这里统一用本函数调用
# 原生命令：内部临时切到 Continue，只把退出码返回给调用方判断。
function Invoke-Native {
    param(
        [Parameter(Mandatory)][string]$FilePath,
        [Parameter(Mandatory)][AllowEmptyCollection()][string[]]$Arguments,
        [Parameter(Mandatory)][string]$LogPath
    )
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $FilePath @Arguments *> $LogPath
        return $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previous
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$harness  = Join-Path $repoRoot 'out\win_outpost_tests'
$buildDir = Join-Path $harness 'build'
$binDir   = Join-Path $buildDir $Config
$logDir   = Join-Path $harness 'logs'
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

$targets = @('outpost_helper_test', 'outpost_geometry_test', 'outpost_target_test', 'outpost_aimer_test')

Write-Host '=============================================================='
Write-Host ' 前哨站测试（Windows / MSVC）'
Write-Host "   仓库根目录 : $repoRoot"
Write-Host "   测试工程   : $harness"
Write-Host "   构建目录   : $buildDir"
Write-Host "   配置       : $Config"
Write-Host '=============================================================='

# ---------------------------------------------------------------------------
# 0. 找工具链与依赖
# ---------------------------------------------------------------------------
function Find-CMake {
    $cands = @()
    $cands += Get-ChildItem 'C:\Program Files\Microsoft Visual Studio\*\*\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -ExpandProperty FullName
    $onPath = Get-Command cmake -ErrorAction SilentlyContinue
    if ($onPath) { $cands += $onPath.Source }
    $cands | Select-Object -First 1
}

function Find-Dep([string]$explicit, [string[]]$candidates, [string]$probe) {
    if ($explicit) {
        if (Test-Path (Join-Path $explicit $probe)) { return $explicit }
        throw "指定的路径下找不到 $probe ：$explicit"
    }
    foreach ($c in $candidates) {
        if ($c -and (Test-Path (Join-Path $c $probe))) { return $c }
    }
    return $null
}

$cmake = Find-CMake
if (-not $cmake) { Write-Host '[ERROR] 找不到 cmake（安装 VS 的 C++ 工作负载或单独安装 CMake）'; exit 2 }

$vcpkg = Find-Dep $VcpkgRoot @(
    $(if ($env:VCPKG_ROOT) { Join-Path $env:VCPKG_ROOT 'installed\x64-windows' }),
    'C:\Users\18916\vcpkg\installed\x64-windows',
    'C:\vcpkg\installed\x64-windows',
    'D:\vcpkg\installed\x64-windows'
) 'share\opencv4\OpenCVConfig.cmake'

$conda = Find-Dep $CondaRoot @(
    $(if ($env:CONDA_PREFIX) { Join-Path $env:CONDA_PREFIX 'Library' }),
    'D:\rmapps\Library',
    'C:\Users\18916\anaconda3\Library'
) 'lib\cmake\fmt\fmt-config.cmake'

if (-not $vcpkg) { Write-Host '[ERROR] 找不到 vcpkg 的 OpenCV/Eigen（可用 -VcpkgRoot 指定 …\vcpkg\installed\x64-windows）'; exit 2 }
if (-not $conda) { Write-Host '[ERROR] 找不到 fmt/spdlog/yaml-cpp（可用 -CondaRoot 指定 …\Library）'; exit 2 }

Write-Host "   cmake      : $cmake"
Write-Host "   OpenCV/Eigen: $vcpkg"
Write-Host "   fmt/…      : $conda"

# ---------------------------------------------------------------------------
# 1. 配置 + 编译
# ---------------------------------------------------------------------------
if ($Clean -and (Test-Path $buildDir)) {
    Write-Host "[1/3] -Clean：删除 $buildDir"
    Remove-Item -Recurse -Force $buildDir
}

$cmakeArgs = @(
    '-S', $harness,
    '-B', $buildDir,
    '-G', $Generator,
    '-A', 'x64',
    "-DCMAKE_PREFIX_PATH=$vcpkg;$conda",
    "-DOpenCV_DIR=$($vcpkg -replace '\\', '/')/share/opencv4",
    "-DEigen3_DIR=$($vcpkg -replace '\\', '/')/share/eigen3",
    "-Dfmt_DIR=$($conda -replace '\\', '/')/lib/cmake/fmt",
    "-Dspdlog_DIR=$($conda -replace '\\', '/')/lib/cmake/spdlog",
    "-Dyaml-cpp_DIR=$($conda -replace '\\', '/')/lib/cmake/yaml-cpp"
)

Write-Host '[1/3] 配置 CMake…'
$configureLog = Join-Path $logDir 'configure.log'
$configureCode = Invoke-Native -FilePath $cmake -Arguments $cmakeArgs -LogPath $configureLog
Get-Content $configureLog -Tail 6 | ForEach-Object { Write-Host "   $_" }
if ($configureCode -ne 0) {
    Write-Host "[ERROR] 配置失败，完整日志：$configureLog"
    exit 2
}

Write-Host "[1/3] 编译：$($targets -join ' ')"
$buildLog = Join-Path $logDir 'build.log'
$buildCode = Invoke-Native -FilePath $cmake `
    -Arguments @('--build', $buildDir, '--config', $Config, '--parallel') -LogPath $buildLog
Select-String -Path $buildLog -Pattern 'error|\.exe' |
    Select-Object -Last 10 | ForEach-Object { Write-Host "   $($_.Line.Trim())" }
if ($buildCode -ne 0) {
    Write-Host "[ERROR] 编译失败，完整日志：$buildLog"
    exit 2
}

# ---------------------------------------------------------------------------
# 2. 运行测试
# ---------------------------------------------------------------------------
# conda/vcpkg 是动态库：运行时需要 DLL 在 PATH 上
$env:PATH = "$vcpkg\bin;$conda\bin;$env:PATH"

Write-Host ''
Write-Host '[2/3] 运行测试'
$results = @()
foreach ($t in $targets) {
    $exe = Join-Path $binDir "$t.exe"
    $log = Join-Path $logDir "$t.log"
    if (-not (Test-Path $exe)) {
        Write-Host "  - $t : [缺失可执行文件]"
        $results += [pscustomobject]@{ Test = $t; Result = 'MISSING'; Detail = $exe }
        continue
    }

    if ($t -eq 'outpost_aimer_test') {
        $code = Invoke-Native -FilePath $exe -Arguments @($ConfigPath) -LogPath $log
    } else {
        $code = Invoke-Native -FilePath $exe -Arguments @() -LogPath $log
    }

    Write-Host "  - $t"
    Get-Content $log -Tail 6 | ForEach-Object { Write-Host "      $_" }

    $detail = (Select-String -Path $log -Pattern '\d+ passed, \d+ failed' | Select-Object -Last 1).Matches.Value
    if (-not $detail) {
        $detail = (Select-String -Path $log -Pattern 'PASS: \d+, FAIL: \d+' | Select-Object -Last 1).Matches.Value
    }
    if (-not $detail) { $detail = "exit $code" }

    $results += [pscustomobject]@{
        Test   = $t
        Result = $(if ($code -eq 0) { 'PASS' } else { 'FAIL' })
        Detail = $detail
    }
}

# ---------------------------------------------------------------------------
# 3. 汇总
# ---------------------------------------------------------------------------
Write-Host ''
Write-Host '=============================================================='
Write-Host ' 汇总'
Write-Host '=============================================================='
$results | Format-Table -AutoSize | Out-String | Write-Host
Write-Host " 日志目录：$logDir"
$failed = ($results | Where-Object { $_.Result -ne 'PASS' }).Count
if ($failed -eq 0) {
    Write-Host ' 结果：全部通过'
    exit 0
} else {
    Write-Host " 结果：$failed 项失败"
    exit 1
}
