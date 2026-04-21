param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("export", "run")]
    [string]$Mode,

    [Parameter(Mandatory = $true)]
    [string]$InputPath,

    [string]$Port,
    [int]$TimeMs = 120,
    [int]$Baudrate = 9600,
    [double]$Timeout = 1.0,
    [double]$SettleTime = 0.0
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$OutputDir = Join-Path $Root "output"
$LogsDir = Join-Path $Root "logs"
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
New-Item -ItemType Directory -Force -Path $LogsDir | Out-Null

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$ResolvedInput = (Resolve-Path $InputPath).Path
$BaseName = [System.IO.Path]::GetFileNameWithoutExtension($ResolvedInput)

if ($Mode -eq "export") {
    $outputFile = Join-Path $OutputDir "${BaseName}_${stamp}_servo.json"
    $logFile = Join-Path $LogsDir "export_${stamp}.log"
    & python -m delta_gcode_servo export-servo-commands $ResolvedInput -o $outputFile --time-ms $TimeMs 2>&1 | Tee-Object -FilePath $logFile
    Write-Host "JSON output: $outputFile"
    Write-Host "Log file: $logFile"
    exit $LASTEXITCODE
}

if (-not $Port) {
    throw "Mode=run requires -Port"
}

$logFile = Join-Path $LogsDir "run_${stamp}.log"
& python -m delta_gcode_servo run-gcode $ResolvedInput --port $Port --baudrate $Baudrate --timeout $Timeout --time-ms $TimeMs --settle-time $SettleTime 2>&1 | Tee-Object -FilePath $logFile
Write-Host "Log file: $logFile"
exit $LASTEXITCODE
