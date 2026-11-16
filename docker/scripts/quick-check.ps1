# Local read-only connectivity checks. Run from project/docker in PowerShell.
$ErrorActionPreference = 'Stop'
docker compose ps
docker compose exec sim cefiro-run ping -c 2 observer
docker compose exec sim cefiro-run ping -c 2 8.8.8.8
docker compose exec sim cefiro-run curl -I --max-time 10 https://www.google.com
Write-Host "Pi ICMP may fail even when Zenoh TCP works:"
docker compose exec sim cefiro-run ping -c 2 192.168.68.59
Write-Host "Zenoh TCP endpoint check (Pi bridge must be running):"
Test-NetConnection 192.168.68.59 -Port 7447
