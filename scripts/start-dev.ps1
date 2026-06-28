# start-dev.ps1 - launch the ROS-free dev web UI (personality + TTS) on this PC.
# Double-click it, or run:  powershell -ExecutionPolicy Bypass -File scripts\start-dev.ps1
# Optional args are passed through to dev_webui.py, e.g.:
#   .\scripts\start-dev.ps1 --idle-secs 10        (faster beats)
#   .\scripts\start-dev.ps1                        (default: --behavior on)

$ErrorActionPreference = "Stop"
$root   = Split-Path -Parent $PSScriptRoot          # repo root (scripts\ -> ..)
$script = Join-Path $PSScriptRoot "dev_webui.py"
$keyFile = Join-Path $PSScriptRoot ".openrouter_key"

# --- API key: prefer the environment, else the gitignored scripts\.openrouter_key ---
if (-not $env:OPENROUTER_API_KEY) {
    if (Test-Path $keyFile) {
        $env:OPENROUTER_API_KEY = (Get-Content $keyFile -Raw).Trim()
    } else {
        Write-Warning "No OPENROUTER_API_KEY and no scripts\.openrouter_key - the AI card will show 'unavailable' (TTS still works)."
    }
}

# --- find a real Python (not the Windows Store stub) ---
$py = $null
foreach ($c in @(
    "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe",
    "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe",
    "$env:LOCALAPPDATA\Programs\Python\Python313\python.exe"
)) { if (Test-Path $c) { $py = $c; break } }
if (-not $py) {
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd -and $cmd.Source -notlike "*WindowsApps*") { $py = $cmd.Source }
}
if (-not $py) { throw "No real Python found. Install Python 3.12 from python.org." }

# --- ensure the phrase bank exists / is current (instant offline beat lines) ---
# Empty/drifted bank -> every body beat goes LIVE -> single-flight guard skips the rest ->
# silences + "no lines (kept old)" spam. --if-needed is a no-op when current and never blocks.
$pregen = Join-Path $PSScriptRoot "pregenerate_phrases.py"
Write-Host "Checking phrase bank (devstate\phrases.json)..." -ForegroundColor Cyan
& $py $pregen "--if-needed"

# default to the full enriched-behaviour loop unless the caller passes their own flags
$passthru = $args
if ($passthru.Count -eq 0) { $passthru = @("--behavior") }

Write-Host "Starting dev web UI ->  http://localhost:8080   (Ctrl+C to stop)" -ForegroundColor Cyan
& $py $script @passthru
