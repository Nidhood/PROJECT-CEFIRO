# Run from PROJECT-CEFIRO\docker with Windows PowerShell 5.1 or newer.
$ErrorActionPreference = "Stop"
if (-not (Test-Path ".env")) { Copy-Item ".env.example" ".env" }
if (-not (Select-String -Path ".env" -Pattern "^XPRA_PASSWORD=" -Quiet)) {
    $bytes = New-Object byte[] 16
    $rng = [System.Security.Cryptography.RandomNumberGenerator]::Create()
    try { $rng.GetBytes($bytes) } finally { $rng.Dispose() }
    $pass = [BitConverter]::ToString($bytes).Replace("-", "")
    Add-Content -Path ".env" -Value "XPRA_PASSWORD=$pass"
    Write-Host "Created XPRA_PASSWORD in .env (do not commit this file)."
} else {
    Write-Host "XPRA_PASSWORD already exists in .env."
}
