param(
    [string]$InterfaceAlias,
    [string]$IpAddress = "192.168.2.100",
    [int]$PrefixLength = 24,
    [switch]$Apply
)

$ErrorActionPreference = "Stop"

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Show-Adapters {
    Write-Host "Available adapters:" -ForegroundColor Cyan
    Get-NetAdapter |
        Sort-Object Status, Name |
        Select-Object Name, InterfaceDescription, Status, MacAddress |
        Format-Table -AutoSize
}

function Get-RecommendedAdapter {
    Get-NetAdapter |
        Where-Object {
            $_.Name -notmatch "Wi-Fi|Wireless|Bluetooth" -and
            $_.InterfaceDescription -notmatch "Wi-Fi|Wireless|Bluetooth|Virtual|VPN|PANGP"
        } |
        Sort-Object @{Expression = { $_.Status -eq "Up" }; Descending = $true }, Name |
        Select-Object -First 1
}

if (-not $InterfaceAlias) {
    Show-Adapters
    $recommended = Get-RecommendedAdapter
    Write-Host ""
    if ($recommended) {
        Write-Host "Recommended adapter:" -ForegroundColor Green -NoNewline
        Write-Host " $($recommended.Name)"
        Write-Host "Suggested apply command:" -ForegroundColor Green
        Write-Host "  .\windows\Set-MG400StaticIp.ps1 -InterfaceAlias '$($recommended.Name)' -Apply"
        Write-Host ""
    }
    Write-Host "Dry run only. Re-run with:" -ForegroundColor Yellow
    Write-Host "  .\windows\Set-MG400StaticIp.ps1 -InterfaceAlias '<EthernetName>' -Apply"
    exit 0
}

Write-Host "Target adapter : $InterfaceAlias"
Write-Host "Target IPv4    : $IpAddress/$PrefixLength"

if (-not $Apply) {
    Write-Host ""
    Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Select-Object IPAddress, PrefixLength |
        Format-Table -AutoSize
    Write-Host ""
    Write-Host "Dry run only. To apply the direct-connect MG400 configuration:" -ForegroundColor Yellow
    Write-Host "  .\windows\Set-MG400StaticIp.ps1 -InterfaceAlias '$InterfaceAlias' -Apply"
    exit 0
}

if (-not (Test-IsAdministrator)) {
    Write-Host ""
    Write-Host "Administrator privileges are required to change adapter IPv4 settings." -ForegroundColor Red
    Write-Host "Open PowerShell with 'Run as administrator', then run:" -ForegroundColor Yellow
    Write-Host "  Set-Location '$PWD'"
    Write-Host "  .\windows\Set-MG400StaticIp.ps1 -InterfaceAlias '$InterfaceAlias' -Apply"
    exit 1
}

$adapter = Get-NetAdapter -InterfaceAlias $InterfaceAlias -ErrorAction Stop
Write-Host "Configuring $($adapter.Name) ..." -ForegroundColor Cyan

Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue

Get-NetRoute -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Remove-NetRoute -Confirm:$false -ErrorAction SilentlyContinue

Set-NetIPInterface -InterfaceAlias $InterfaceAlias -Dhcp Disabled -ErrorAction SilentlyContinue | Out-Null
New-NetIPAddress -InterfaceAlias $InterfaceAlias -IPAddress $IpAddress -PrefixLength $PrefixLength -ErrorAction Stop | Out-Null

Write-Host ""
Write-Host "Done. Direct-connect settings applied." -ForegroundColor Green
Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Select-Object IPAddress, PrefixLength |
    Format-Table -AutoSize
Write-Host ""
Write-Host "Next steps:"
Write-Host "  1. Connect the MG400 Ethernet cable directly to this adapter"
Write-Host "  2. Power the robot"
Write-Host "  3. Verify with: ping 192.168.2.7"
Write-Host "     The first ping can lose one packet while Windows finishes applying the address"
Write-Host "  4. Run: python mg400\01_connect_test.py"
