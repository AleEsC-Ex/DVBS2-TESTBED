<#
.SYNOPSIS
    Stops the DVB-S2 testbed's MATLAB windows -- either a run still inside
    its configured duration, or leftover windows from one that already
    finished (deliberately left open so results can be read).

    Deliberately does NOT do "Stop-Process -Name matlab" -- that would also
    kill any unrelated MATLAB window you might have open for something
    else. Instead it identifies each testbed process by COMMAND LINE --
    specifically, which script it was told to run -- and stops only those.

.WHY NOT THE PID Launch-DVBS2Testbed.ps1 USED TO RECORD
    matlab.exe's command-line launcher is a short-lived BOOTSTRAP process:
    it spawns the real, long-running MATLAB.exe engine as a CHILD process
    (a different PID) and then exits itself, often within a second or two.
    Start-Process -PassThru only ever sees that bootstrap PID. By the time
    anything tries to stop it later, it is already gone -- Get-Process
    correctly reports it "already exited", while the real engine (the
    child, still running, never recorded anywhere) is left completely
    untouched. Measured directly: every PID a previous version of
    Launch-DVBS2Testbed.ps1 recorded was dead within seconds of being
    written, which is why stopping a run this way looked like it did
    nothing no matter how many times it was run. Matching on command line
    instead sidesteps the whole bootstrap/engine indirection -- it finds
    whichever PID is CURRENTLY running a given script, however many layers
    of process spawning sit above it.
#>

$ErrorActionPreference = 'Stop'

# Matches Launch-DVBS2Testbed.ps1's $scripts.File values. Kept as its own
# list rather than re-derived from that file, so this script has no
# dependency on Launch's internal variable layout -- update both if a
# script is ever renamed or added.
$scriptNames = @(
    'S1a_Transmitter.m'
    'S1b_ACMControl.m'
    'S2a_RFAcquisition.m'
    'S2b_Reciever.m'
    'S3_ProcessingUnit.m'
)

$matlabProcs = Get-CimInstance Win32_Process -Filter "Name='MATLAB.exe' or Name='matlab.exe'" -ErrorAction SilentlyContinue
if (-not $matlabProcs) {
    Write-Host "No MATLAB processes running -- nothing to stop." -ForegroundColor DarkGray
    exit
}

$stopped = 0
foreach ($name in $scriptNames) {
    # Substring match against the full command line ("...-r "run('...\S1a_
    # Transmitter.m')""), so it doesn't matter what path the script lives
    # at or what other flags surround it -- only that THIS script name
    # appears. A MATLAB engine's own internal helper subprocesses (the
    # "-catapultTransport" worker every engine spawns) never carry a
    # script name in their command line, so they never match and are left
    # alone; killing the engine process they belong to is enough.
    $found = $matlabProcs | Where-Object { $_.CommandLine -and $_.CommandLine.Contains($name) }
    foreach ($p in $found) {
        try {
            Stop-Process -Id $p.ProcessId -Force -ErrorAction Stop
            Write-Host "stopped $name (PID $($p.ProcessId))" -ForegroundColor Yellow
            $stopped++
        } catch {
            Write-Host "could not stop $name (PID $($p.ProcessId)): $($_.Exception.Message)" -ForegroundColor Red
        }
    }
}

if ($stopped -eq 0) {
    Write-Host "No testbed MATLAB windows found running." -ForegroundColor DarkGray
}
