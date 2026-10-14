param(
    [string]$Volume = 'autonav-full-track-20260928',
    [string]$Image = 'autonav-humble-fortress:20260928',
    [int]$MissionTimeoutSec = 90,
    [string]$RunName = 'compact-ros-smoke',
    [int]$ViewerPort = 8765,
    [switch]$NoViewer,
    [switch]$FullTrack
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
if ($FullTrack) {
    if (-not $PSBoundParameters.ContainsKey('MissionTimeoutSec')) { $MissionTimeoutSec = 900 }
    if (-not $PSBoundParameters.ContainsKey('RunName')) {
        $RunName = 'full-track-' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssZ')
    }
    $runScript = '/host/integrations/ros_humble_fortress/run_full_track.sh'
} else {
    $runScript = '/host/integrations/ros_humble_fortress/run_compact_smoke.sh'
}
if ($RunName -notmatch '^[a-zA-Z0-9_-]+$') {
    throw 'RunName must contain only letters, digits, underscores, or hyphens.'
}
if ($MissionTimeoutSec -lt 1 -or $ViewerPort -lt 1 -or $ViewerPort -gt 65535) {
    throw 'MissionTimeoutSec and ViewerPort must be positive; ViewerPort must be at most 65535.'
}
$dockerArgs = @(
    'run', '--gpus', 'all',
    '-v', "${Volume}:/ws",
    '-v', "${repo}:/host",
    '-e', "MISSION_TIMEOUT_SEC=$MissionTimeoutSec",
    '-e', "RUN_DIR=/host/runs/$RunName"
)

if ($NoViewer) {
    $headlessArgs = $dockerArgs + @('--rm', $Image, 'bash',
        $runScript)
    & docker @headlessArgs
    exit $LASTEXITCODE
}

$dockerArgs += @(
    '-p', "127.0.0.1:${ViewerPort}:8765",
    '-e', 'SPECTATOR_PORT=8765',
    $Image, 'bash', $runScript
)
$containerId = $null
$missionExit = 1
try {
    $detachedArgs = @('run', '-d') + $dockerArgs[1..($dockerArgs.Count - 1)]
    $runOutput = @(& docker @detachedArgs 2>&1)
    if ($LASTEXITCODE -ne 0) {
        throw "Docker could not start the course run: $($runOutput -join ' ')"
    }
    $containerId = $runOutput | Where-Object { "$_" -match '^[0-9a-f]{64}$' } | Select-Object -Last 1
    if (-not $containerId) {
        throw "Docker did not return a container ID: $($runOutput -join ' ')"
    }

    $url = "http://127.0.0.1:$ViewerPort/"
    $ready = $false
    $deadline = [DateTime]::UtcNow.AddSeconds(45)
    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $response = Invoke-WebRequest -Uri $url -TimeoutSec 2 -UseBasicParsing
            if ($response.StatusCode -eq 200) {
                $ready = $true
                break
            }
        } catch {
            Start-Sleep -Milliseconds 500
        }
        $running = & docker inspect --format '{{.State.Running}}' $containerId 2>$null
        if ($running -eq 'false') { break }
    }
    if ($ready) {
        Write-Host "Live course view: $url"
        try {
            Start-Process -FilePath $url
        } catch {
            Write-Warning "Could not open the browser automatically. Open $url yourself."
        }
    } else {
        Write-Warning "The course viewer did not become ready at $url. Check runs/$RunName/viewer.log."
    }

    & docker logs -f $containerId
    $waitOutput = @(& docker wait $containerId)
    if ($LASTEXITCODE -ne 0 -or $waitOutput.Count -ne 1) {
        throw "Docker could not retrieve the mission exit code: $($waitOutput -join ' ')"
    }
    $missionExit = [int]$waitOutput[0]
    $replayPath = Join-Path $repo "runs\$RunName\replay.html"
    if (Test-Path -LiteralPath $replayPath) {
        Write-Host "Saved course replay: $replayPath"
        try {
            Start-Process -FilePath $replayPath
        } catch {
            Write-Warning "Could not open the replay automatically. Open $replayPath yourself."
        }
    }
} finally {
    if ($containerId) {
        & docker rm -f $containerId 2>$null | Out-Null
    }
}
exit $missionExit
