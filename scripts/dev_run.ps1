<#
.SYNOPSIS
  Run the FULL dev web UI on this PC, online (real OpenRouter LLM), with one command:
  autonomous behaviour beats + the Brain card (purpose / pursuing / A/B reward /
  meditation) + the AI card + TTS - everything wired.

.DESCRIPTION
  Loads the OpenRouter key from $env:OPENROUTER_API_KEY, or (if unset) from the gitignored
  scripts\.openrouter_key file (one line - NEVER committed; it's in .gitignore). Then launches
  scripts\dev_webui.py with --behavior. Extra args pass straight through to dev_webui.py.

.EXAMPLE
  scripts\dev_run.ps1                 # full stack, beats every ~15 s
  scripts\dev_run.ps1 --idle-secs 8   # faster beats (see pursuing sooner)
  scripts\dev_run.ps1 --voice de-DE   # German TTS
#>
$ErrorActionPreference = "Stop"

# Repo root = the parent of this scripts\ directory.
$scriptsDir = $PSScriptRoot
$root = Split-Path -Parent $scriptsDir

# --- 1. Resolve the OpenRouter key (env first, then the local gitignored file) ----------
$key = $env:OPENROUTER_API_KEY
$keySrc = "env:OPENROUTER_API_KEY"
if ([string]::IsNullOrWhiteSpace($key)) {
    $keyFile = Join-Path $scriptsDir ".openrouter_key"
    if (Test-Path $keyFile) {
        $key = (Get-Content -Raw -Encoding ascii $keyFile).Trim()
        $keySrc = "scripts\.openrouter_key"
    }
}
if ([string]::IsNullOrWhiteSpace($key)) {
    Write-Host "No OpenRouter key found." -ForegroundColor Red
    Write-Host "  Put your key (one line) in:  $($scriptsDir)\.openrouter_key" -ForegroundColor Yellow
    Write-Host "  or set the OPENROUTER_API_KEY env var first. (.openrouter_key is gitignored.)"
    exit 1
}
$env:OPENROUTER_API_KEY = $key
# Show only that it loaded + a masked tail, never the key itself.
$tail = if ($key.Length -ge 4) { $key.Substring($key.Length - 4) } else { "" }
Write-Host "OpenRouter key loaded from $keySrc (...$tail)" -ForegroundColor Green

# --- 2. Find a real Python (skip the Microsoft Store WindowsApps stub) -------------------
$py = $null
$cmd = Get-Command python -ErrorAction SilentlyContinue
if ($cmd -and $cmd.Source -notlike "*WindowsApps*") { $py = $cmd.Source }
if (-not $py) {
    $cand = Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"
    if (Test-Path $cand) { $py = $cand }
}
if (-not $py) {
    $cmd = Get-Command py -ErrorAction SilentlyContinue
    if ($cmd) { $py = $cmd.Source }
}
if (-not $py) {
    Write-Host "No Python found (looked for: python, %LOCALAPPDATA%\Programs\Python\Python312, py)." -ForegroundColor Red
    exit 1
}

# --- 3. Launch the full dev UI (autonomous beats on) ------------------------------------
$devui = Join-Path $scriptsDir "dev_webui.py"
$argv = @($devui, "--behavior") + $args
Write-Host "Launching:  $py $($argv -join ' ')" -ForegroundColor Cyan
Write-Host "Open http://localhost:8080  ->  Speak tab (AI card + Brain card). Ctrl+C to stop.`n"
& $py @argv
