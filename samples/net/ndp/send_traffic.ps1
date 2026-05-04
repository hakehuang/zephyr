<#
.SYNOPSIS
    Send UDP traffic to the NDP sample running in QEMU

.DESCRIPTION
    Sends UDP packets to 10.0.2.15 (QEMU guest) on port 4242
    to trigger the NDP callback and generate performance reports.

.PARAMETER Count
    Number of packets to send (default: 100)

.PARAMETER PacketSize
    Size of each packet in bytes (default: 64)

.PARAMETER Interval
    Milliseconds between packets (default: 100)

.PARAMETER Port
    UDP port (default: 4242)

.EXAMPLE
    .send_traffic.ps1 -Count 1000 -PacketSize 256
#>

param(
    [int]$Count = 100,
    [int]$PacketSize = 64,
    [int]$Interval = 100,
    [int]$Port = 4242
)

$TargetIP = "10.0.2.15"
$Address = [System.Net.IPAddress]::Parse($TargetIP)
$Endpoint = New-Object System.Net.IPEndPoint $Address, $Port

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "NDP Traffic Generator" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ("Target:    " + $TargetIP + ":" + $Port)
Write-Host ("Packets:   " + $Count)
Write-Host ("Size:      " + $PacketSize + " bytes")
Write-Host ("Interval:  " + $Interval + "ms")
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

$sent = 0
$errors = 0
$startTime = Get-Date

for ($i = 1; $i -le $Count; $i++) {
    try {
        $UdpClient = New-Object System.Net.Sockets.UdpClient
        $Bytes = [byte[]]::new($PacketSize)
        (new-object System.Random).NextBytes($Bytes)
        $UdpClient.Send($Bytes, $PacketSize, $Endpoint) | Out-Null
        $UdpClient.Close()
        $sent++
        
        if ($i % 10 -eq 0) {
            Write-Host ("Sent " + $i + " / " + $Count + " packets...") -ForegroundColor Gray
        }
    }
    catch {
        Write-Host "[ERROR] Packet $i failed: $_" -ForegroundColor Red
        $errors++
    }
    
    if ($Interval -gt 0 -and $i -lt $Count) {
        Start-Sleep -Milliseconds $Interval
    }
}

$elapsed = (Get-Date) - $startTime
$rate = [math]::Round($sent / $elapsed.TotalSeconds)

Write-Host ""
Write-Host "==========================================" -ForegroundColor Green
Write-Host "Complete!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Green
Write-Host ("Sent:      " + $sent + " packets")
Write-Host ("Errors:    " + $errors)
Write-Host ("Duration:  " + $elapsed.TotalSeconds.ToString("F1") + " seconds")
Write-Host ("Rate:      " + $rate + " pkt/s")
Write-Host ""
Write-Host "Check the QEMU console for NDP performance reports."
Write-Host ""
