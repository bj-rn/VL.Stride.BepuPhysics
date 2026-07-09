# Launches vvvv for each help patch and watches its stdout for exceptions (60s window each).
# Usage: powershell -File tools\verify-patches.ps1 [-VvvvExe <path>] [-Seconds 60]
param(
    [string]$VvvvExe = "D:\vvvv\vvvv_gamma_7.3-win-x64\vvvv.exe",
    [int]$Seconds = 60
)

$repoRoot = Split-Path $PSScriptRoot -Parent
$packageRepo = Split-Path $repoRoot -Parent
$helpDir = Join-Path $repoRoot "help"
$failed = $false

$patches = Get-ChildItem $helpDir -Filter *.vl -Recurse
foreach ($patch in $patches) {
    Write-Host "=== $($patch.Name) ===" -ForegroundColor Cyan
    $out = Join-Path $env:TEMP ("vvvv-verify-" + [IO.Path]::GetFileNameWithoutExtension($patch.Name) + ".log")
    $p = Start-Process -FilePath $VvvvExe `
        -ArgumentList @("--package-repositories", $packageRepo, "-o", "`"$($patch.FullName)`"") `
        -RedirectStandardOutput $out -PassThru

    $deadline = (Get-Date).AddSeconds($Seconds)
    $problem = $null
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 5
        if ($p.HasExited) { $problem = "vvvv exited early (code $($p.ExitCode))"; break }
        $hits = Select-String -Path $out -Pattern "Exception|error CS" -SimpleMatch:$false -ErrorAction SilentlyContinue
        if ($hits) { $problem = ($hits | Select-Object -First 3 | ForEach-Object Line) -join "`n"; break }
    }

    if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }

    if ($problem) {
        Write-Host "FAILED: $problem" -ForegroundColor Red
        $failed = $true
    } else {
        Write-Host "OK ($Seconds s, no exceptions on stdout)" -ForegroundColor Green
    }
}

if ($failed) { exit 1 } else { Write-Host "All patches verified." -ForegroundColor Green }
