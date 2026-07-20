# save as: flash-and-erase.ps1
param([string]$port = "COM8")

$pythonPath = "C:\Users\Administrator\AppData\Local\Python\pythoncore-3.14-64\python.exe"
$espIdfPath = "C:\Espressif\esp-idf"  # Adjust if needed

Write-Host "`n" -ForegroundColor Cyan
Write-Host "╔════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║  ESP32-C6 Flash Erase & Program Tool                  ║" -ForegroundColor Green
Write-Host "╚════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# Verify Python exists
if (!(Test-Path $pythonPath)) {
    Write-Host "✗ Python not found at: $pythonPath" -ForegroundColor Red
    exit 1
}

Write-Host "✓ Python: $pythonPath" -ForegroundColor Green
Write-Host "✓ Port: $port" -ForegroundColor Green
Write-Host ""

# Step 1: Erase
Write-Host "╔════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║  Step 1: Erasing Flash Memory                         ║" -ForegroundColor Yellow
Write-Host "╚════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

& $pythonPath -m esptool --chip esp32c6 --port $port erase-flash

if ($LASTEXITCODE -ne 0) {
    Write-Host "`n✗ Flash erase failed!" -ForegroundColor Red
    exit 1
}

Write-Host "`n✓ Flash erased`n" -ForegroundColor Green

# Step 2: Build
Write-Host "╔════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║  Step 2: Building Project                             ║" -ForegroundColor Yellow
Write-Host "╚════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# Use python -m idf instead of idf.py
& $pythonPath -m idf build

if ($LASTEXITCODE -ne 0) {
    Write-Host "`n✗ Build failed!" -ForegroundColor Red
    exit 1
}

Write-Host "`n✓ Build successful`n" -ForegroundColor Green

# Step 3: Flash
Write-Host "╔════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║  Step 3: Programming Flash                            ║" -ForegroundColor Yellow
Write-Host "╚════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

& $pythonPath -m idf -p $port -b 460800 flash

if ($LASTEXITCODE -ne 0) {
    Write-Host "`n✗ Flash failed!" -ForegroundColor Red
    exit 1
}

Write-Host "`n✓ Flash successful`n" -ForegroundColor Green

# Step 4: Monitor
Write-Host "╔════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║  Step 4: Monitoring Output (Ctrl+C to exit)           ║" -ForegroundColor Yellow
Write-Host "╚════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

Start-Sleep -Seconds 2

& $pythonPath -m idf -p $port monitor

Write-Host "`n✅ Done!`n" -ForegroundColor Green