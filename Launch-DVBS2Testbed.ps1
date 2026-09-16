<#
.SYNOPSIS
    Starts every DVB-S2 testbed process, each in its own console-only
    MATLAB window, without you opening MATLAB by hand and typing run(...)
    once per script.
#>

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------
# WHAT TO LAUNCH. Name is only used for window/log labelling; File is the
# script's path relative to the Testbed/ folder (see $codeDir below) --
# all the MATLAB source lives there, separate from this launcher and the
# repo's docs/README at the root.
# ---------------------------------------------------------------------
$scripts = @(
    @{ Name = 'S1a'; File = 'S1a_Transmitter.m' }
    @{ Name = 'S1b'; File = 'S1b_ACMControl.m' }
    @{ Name = 'S2a'; File = 'S2a_RFAcquisition.m' }
    @{ Name = 'S2b'; File = 'S2b_Reciever.m' }
    @{ Name = 'S3';  File = 'S3_ProcessingUnit.m' }
)

# Seconds to wait between opening each window.
$staggerSeconds = 1

# ---------------------------------------------------------------------
# Locate matlab.exe. Prefers whatever is already on PATH; falls back to
# scanning the usual install location so this still works on a machine
# where MATLAB was never added to PATH.
# ---------------------------------------------------------------------
function Find-MatlabExe {
    $onPath = Get-Command matlab.exe -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }

    $candidates = Get-ChildItem -Path 'C:\Program Files\MATLAB' -Filter 'matlab.exe' `
        -Recurse -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending
    if ($candidates) { return $candidates[0].FullName }

    throw "Could not find matlab.exe on PATH or under C:\Program Files\MATLAB. " +
          "Edit `$matlabExe at the top of this script with the correct path."
}

$projectRoot = $PSScriptRoot
$codeDir     = Join-Path $projectRoot 'Testbed'
$matlabExe   = Find-MatlabExe
$logDir      = Join-Path $projectRoot 'logs'
if (-not (Test-Path $logDir)) {
    New-Item -ItemType Directory -Path $logDir | Out-Null
}

$timestamp = Get-Date -Format 'yyyy-MM-dd_HHmmss'

Write-Host "=== DVB-S2 testbed launcher ===" -ForegroundColor Cyan
Write-Host "MATLAB:  $matlabExe"
Write-Host "Logs:    $logDir\*_$timestamp.log"
Write-Host ""

# ---------------------------------------------------------------------
# CLOSE LEFTOVER WINDOWS FROM THE PREVIOUS RUN, if any are still open.
# Windows are deliberately left open after a run finishes (see the "no
# exit" note below) so you can read the results -- which means a port
# from last time can still be genuinely held, not just in the OS's brief
# post-close TIME_WAIT state, when you start the next run. The retry
# loops in dvbs2TCPServerRetry.m only ride out TIME_WAIT; they can't do
# anything about a window that's simply still open. Doing the cleanup
# HERE, not at the end of the previous run, is what lets you actually
# read that previous run's results before this happens.
$stopScript = Join-Path $PSScriptRoot 'Stop-DVBS2Testbed.ps1'
if (Test-Path $stopScript) {
    & $stopScript
    Start-Sleep -Seconds 2   # let the OS actually release the closed sockets
    Write-Host ""
}

foreach ($s in $scripts) {
    $scriptPath = Join-Path $codeDir $s.File
    if (-not (Test-Path $scriptPath)) {
        Write-Warning "Skipping $($s.Name): $scriptPath not found."
        continue
    }

    $logPath = Join-Path $logDir "$($s.Name)_$timestamp.log"
    # run(...) instead of just naming the script, so the working directory
    # MATLAB starts in doesn't matter -- the full path is explicit.
    #
    # DELIBERATELY NO "; exit" HERE. That was tried and reverted -- it
    # closed each window the instant its script finished, taking the
    # on-screen results with it before they could be read. S2b's actual
    # stop bug is fixed at the source now (its chunk-read loop has its
    # own duration check, so it no longer depends on S2a's socket ever
    # closing), so nothing here needs the window to disappear. Leftover
    # windows from a PREVIOUS run are handled instead, below, before this
    # run starts -- that's the point in the lifecycle where stale sockets
    # actually cause a problem (a new run failing to bind an already-held
    # port), not after a run finishes.
    $runCmd = "run('$scriptPath')"

    $proc = Start-Process -FilePath $matlabExe `
        -ArgumentList @('-nodesktop', '-nosplash', '-logfile', "`"$logPath`"", '-r', "`"$runCmd`"") `
        -PassThru

    # This PID is the short-lived bootstrap process, not the real MATLAB
    # engine that ends up running the script (see the docstring above) --
    # printed only as a bring-up sanity check, not something anything else
    # in this testbed relies on to identify the process later.
    Write-Host ("  started {0,-4} (bootstrap PID {1,-6}) -- log: {2}" -f $s.Name, $proc.Id, $logPath) -ForegroundColor Green

    Start-Sleep -Seconds $staggerSeconds
}

Write-Host ""
Write-Host "All windows started. Each stops on its own after its configured run duration." -ForegroundColor Cyan
Write-Host "To stop everything early, run:  .\Stop-DVBS2Testbed.ps1"
