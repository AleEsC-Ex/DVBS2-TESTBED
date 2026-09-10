<#
.SYNOPSIS
    Stops exactly the MATLAB processes that Launch-DVBS2Testbed.ps1 started
    last, using the PID list it saved to logs\last-run-pids.txt.

    Deliberately does NOT do "Stop-Process -Name matlab" -- that would also
    kill any unrelated MATLAB window you might have open for something else.
#>

$ErrorActionPreference = 'Stop'

$pidFile = Join-Path $PSScriptRoot 'logs\last-run-pids.txt'
if (-not (Test-Path $pidFile)) {
    Write-Warning "No $pidFile found -- nothing to stop (has the launcher been run yet?)."
    exit
}

$entries = Get-Content $pidFile | Where-Object { $_ -match '=' }
if (-not $entries) {
    Write-Warning "$pidFile is empty."
    exit
}

foreach ($line in $entries) {
    $name, $procId = $line -split '=', 2
    $proc = Get-Process -Id $procId -ErrorAction SilentlyContinue
    if ($proc) {
        Stop-Process -Id $procId -Force
        Write-Host "stopped $name (PID $procId)" -ForegroundColor Yellow
    } else {
        Write-Host "$name (PID $procId) already exited" -ForegroundColor DarkGray
    }
}
