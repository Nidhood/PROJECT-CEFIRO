# PowerShell script: opt-in fix for CRLF shebangs in local CEFIRO Python scripts.
# Run from project/docker as: .\scripts\fix-ros-python-shebangs.ps1
# Changes only Python files whose first line uses /usr/bin/env and which contain CRLF.
$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$files = Get-ChildItem -Path $repositoryRoot -Recurse -File -Filter "*.py" |
    Where-Object { $_.FullName -notmatch '[\\/](\.git|\.docker-data|build|install|log)[\\/]' }
foreach ($file in $files) {
    $content = [System.IO.File]::ReadAllText($file.FullName)
    $firstLine = ($content -split "`n", 2)[0]
    if ($firstLine.StartsWith('#!/usr/bin/env ') -and $content.Contains("`r`n")) {
        $fixed = $content.Replace("`r`n", "`n")
        [System.IO.File]::WriteAllText($file.FullName, $fixed, [System.Text.UTF8Encoding]::new($false))
        Write-Host "Converted to LF: $($file.FullName)"
    }
}
