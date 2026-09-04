$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$TargetDir = Join-Path $ProjectRoot "MDK-ARM\STM32F103_inclinometer  - 4G - V3.3.1"
$ExpectedParent = Join-Path $ProjectRoot "MDK-ARM"

$ResolvedRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
$ResolvedParent = (Resolve-Path -LiteralPath $ExpectedParent).Path

if (-not (Test-Path -LiteralPath $TargetDir -PathType Container)) {
    Write-Host "Keil输出目录不存在，不需要清理："
    Write-Host $TargetDir
    Write-Host ""
    Read-Host "按回车退出"
    exit 0
}

$ResolvedTarget = (Resolve-Path -LiteralPath $TargetDir).Path
if (-not $ResolvedTarget.StartsWith($ResolvedParent, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "安全校验失败：目标目录不在当前工程 MDK-ARM 目录内。"
}

$AllowedExtensions = @(
    ".o", ".d", ".dep", ".axf", ".elf", ".hex", ".bin", ".s19",
    ".map", ".htm", ".lnp", ".crf", ".lst", ".build_log.htm"
)

$Files = Get-ChildItem -LiteralPath $ResolvedTarget -File | Where-Object {
    $name = $_.Name.ToLowerInvariant()
    ($AllowedExtensions | Where-Object { $name.EndsWith($_) }).Count -gt 0
}

Write-Host "工程目录：$ResolvedRoot"
Write-Host "清理目录：$ResolvedTarget"
Write-Host ""

if ($Files.Count -eq 0) {
    Write-Host "没有找到需要清理的 Keil 编译产物。"
    Write-Host ""
    Read-Host "按回车退出"
    exit 0
}

Write-Host "将删除以下 $($Files.Count) 个文件："
foreach ($file in $Files) {
    Write-Host "  $($file.Name)"
}
Write-Host ""

foreach ($file in $Files) {
    Remove-Item -LiteralPath $file.FullName -Force
}

Write-Host "清理完成。"
Write-Host "保留内容：Keil工程文件、EIDE配置、源码、Drivers、dist工具、文档。"
Write-Host ""
Read-Host "按回车退出"
