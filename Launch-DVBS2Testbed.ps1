<#
.SYNOPSIS
    Starts every DVB-S2 testbed process, each in its own console-only
    MATLAB window, without you opening MATLAB by hand and typing run(...)
    once per script.

.HOW THIS WORKS, FOR SOMEONE WHO HASN'T SCRIPTED THIS BEFORE
    Every S*.m script already finds its own folder via mfilename('fullpath')
    and adds Testbed/Functions/ to its own path, and every inter-process
    link uses a connect-with-retry helper (dvbs2TCPConnectRetry /
    dvbs2TCPServerRetry). That means STARTUP ORDER DOES NOT MATTER -- this
    script can open all five windows back to back with no coordination
    logic, and each one just waits until its counterpart shows up on the
    expected port.

    All this script does, five times over, is:
      1. build the command line MATLAB needs ( -r "run('...')" )
      2. hand it to Start-Process, which opens a new window and returns
         immediately (it does NOT wait for that window to finish)

    Stopping a window later (Stop-DVBS2Testbed.ps1) does NOT use the PID
    Start-Process returns here -- that PID belongs to a short-lived
    bootstrap process which hands off to the real, long-running MATLAB
    engine as a CHILD process and then exits within a second or two, so by
    the time anything tries to stop it, it is already gone. Stop-
    DVBS2Testbed.ps1 finds the real engine by matching its command line
    against each script's name instead; see that file's own header for the
    full story.

.USAGE
    Just run this file from PowerShell:  .\Launch-DVBS2Testbed.ps1
    To add the planned fifth process once it exists, uncomment its line
    in $scripts below -- nothing else in this file needs to change.
#>

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------
# WHAT TO LAUNCH, AND IN WHAT ORDER. Name is only used for window/log
# labelling; File is the script's path relative to the Testbed/ folder
# (see $codeDir below) -- all the MATLAB source lives there, separate from
# this launcher and the repo's docs/README at the root.
#
# Startup order does not affect CORRECTNESS -- every process retries its
# own connections regardless of who else is up yet -- but it does affect
# how much of the stagger's headroom each one actually gets, since it's
# spent between launches, not evenly. S2a and S2b are ordered first
# because they have the most expensive cold start (radio construction,
# USRP driver init) and benefit most from a head start; S1a/S1b follow;
# S3 -- the cheapest, fastest process to initialize -- goes last, since it
# needs the least of the stagger's headroom to be ready in time.
# ---------------------------------------------------------------------
$scripts = @(
    @{ Name = 'S2a'; File = 'S2a_RFAcquisition.m' }
    @{ Name = 'S2b'; File = 'S2b_Reciever.m' }
    @{ Name = 'S1a'; File = 'S1a_Transmitter.m' }
    @{ Name = 'S1b'; File = 'S1b_ACMControl.m' }
    @{ Name = 'S3';  File = 'S3_ProcessingUnit.m' }
)

# Seconds to wait between opening each window. Not needed for correctness
# (every script retries its own connections) -- it's what's actually fixing
# the resource contention diagnosed on the 2026-09-08 13:16 run: four MATLAB
# processes cold-starting within ~4.5 s of each other produced 214 RX
# overruns, 25 TX underruns and 10.8% frame loss, none of it from a code bug
# -- purely from four license checks / JIT compiles / USRP driver inits
# competing for the same CPU cores at once. 15 s spaces that out so each
# process's expensive one-time startup cost has mostly settled before the
# next one begins competing for it. (Briefly lowered to 5 s on 2026-09-11
# on the reasoning that every process now blocks on its own downstream
# connections before doing real work, so a shorter stagger couldn't cause
# a correctness problem -- restored to 15 s per instruction, combined with
# reordering $scripts above so the slowest-starting processes get launched
# first and get the most benefit from whatever stagger there is.)
$staggerSeconds = 15

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
