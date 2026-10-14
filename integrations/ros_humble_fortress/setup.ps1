param(
    [string]$Volume = 'autonav-full-track-20260928',
    [string]$Image = 'autonav-humble-fortress:20260928'
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
docker build -t $Image -f (Join-Path $PSScriptRoot 'Dockerfile') $repo
if ($LASTEXITCODE -ne 0) { throw 'Docker image build failed.' }
docker volume create $Volume | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Docker volume creation failed.' }
$dockerArgs = @(
    'run', '--rm',
    '-v', "${Volume}:/ws",
    '-v', "${repo}:/host",
    $Image, 'bash',
    '/host/integrations/ros_humble_fortress/bootstrap_workspace.sh'
)
docker @dockerArgs
if ($LASTEXITCODE -ne 0) { throw 'Pinned ROS workspace build failed.' }
