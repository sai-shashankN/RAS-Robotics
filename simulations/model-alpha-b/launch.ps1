param(
    [ValidateSet('geometry', 'dynamics', 'grasp', 'grasp_negative')][string]$Mode = 'geometry',
    [switch]$Validate,
    [ValidateSet('balance','tilt_positive','tilt_negative','push','steering','drive','arm_shoulder','arm_elbow','arm_wrist','arm_reach','pose_guard','claw_cycle_folded','claw_cycle_raised')][string]$Scenario = 'balance',
    [string]$WebotsPath
)
$ErrorActionPreference = 'Stop'
$workspaceRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
if (-not $WebotsPath) {
    $WebotsPath = Join-Path $workspaceRoot '.local-tools\webots\msys64\mingw64\bin\webots.exe'
}
if (-not (Test-Path -LiteralPath $WebotsPath)) {
    throw 'Webots not found. Supply -WebotsPath or install Webots R2025a.'
}
$modelName = (Get-Content -LiteralPath (Join-Path $PSScriptRoot 'parameters.json') -Raw | ConvertFrom-Json).model.ToLower()
$scenarioSuffix = $(if ($Validate -and $Mode -eq 'dynamics' -and $Scenario -ne 'balance') { '_' + $Scenario } else { '' })
$worldName = $modelName + '_' + $Mode + $scenarioSuffix + $(if ($Validate) { '_validation' } else { '' }) + '.wbt'
$worldPath = Join-Path $PSScriptRoot ('worlds\' + $worldName)
if (-not (Test-Path -LiteralPath $worldPath)) { throw 'Generate worlds with scripts/generate_worlds.py first.' }
$webotsBinDirectory = Split-Path -Parent $WebotsPath
$webotsInstallRoot = (Resolve-Path -LiteralPath (Join-Path $webotsBinDirectory '..\..\..')).Path
$previousPath = $env:PATH
$previousWebotsHome = $env:WEBOTS_HOME
try {
    # R2025a's launcher crashes here with the inherited ~4 KB PATH. Scope a short
    # PATH to this child process; never modify the user's persistent environment.
    $env:PATH = $webotsBinDirectory + ';' + (Join-Path $webotsInstallRoot 'msys64\usr\bin') + ';' + (Join-Path $env:SystemRoot 'System32') + ';' + $env:SystemRoot + ';C:\Python313'
    $env:WEBOTS_HOME = $webotsInstallRoot
    $arguments = @('--mode=realtime', ('"' + $worldPath + '"'))
    if ($Validate) {
        $resultDirectory = Join-Path $PSScriptRoot ('results\' + $Mode)
        if ($Mode -eq 'dynamics' -and $Scenario -ne 'balance') { $resultDirectory = Join-Path $resultDirectory $Scenario }
        New-Item -ItemType Directory -Path $resultDirectory -Force | Out-Null
        $arguments = @('--batch', '--mode=fast', '--stdout', '--stderr', ('"' + $worldPath + '"'))
        $simulatorProcess = Start-Process -FilePath $WebotsPath -ArgumentList $arguments -WindowStyle Hidden -PassThru -Wait -RedirectStandardOutput (Join-Path $resultDirectory 'webots-stdout.log') -RedirectStandardError (Join-Path $resultDirectory 'webots-stderr.log')
        $validationExit = $simulatorProcess.ExitCode
    } else {
        # Interactive opening is the explicit purpose when the user runs this script.
        Start-Process -FilePath $WebotsPath -ArgumentList $arguments
    }
} finally {
    $env:PATH = $previousPath
    $env:WEBOTS_HOME = $previousWebotsHome
}
if ($Validate) { exit $validationExit }

