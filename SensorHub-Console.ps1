# Sensor Hub Serial Console for PowerShell ISE
# Save as: SensorHub-Console.ps1

param(
    [string]$port = "COM8",
    [int]$baudRate = 115200
)

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# Create Form
$form = New-Object System.Windows.Forms.Form
$form.Text = "Sensor Hub Console - $port @ $baudRate baud"
$form.Width = 1200
$form.Height = 750
$form.StartPosition = "CenterScreen"
$form.Font = New-Object System.Drawing.Font("Segoe UI", 10)

# Output TextBox
$outputBox = New-Object System.Windows.Forms.TextBox
$outputBox.Location = New-Object System.Drawing.Point(10, 10)
$outputBox.Size = New-Object System.Drawing.Size(1170, 500)
$outputBox.Multiline = $true
$outputBox.ReadOnly = $true
$outputBox.ScrollBars = "Vertical"
$outputBox.Font = New-Object System.Drawing.Font("Courier New", 9)
$form.Controls.Add($outputBox)

# Input TextBox
$inputBox = New-Object System.Windows.Forms.TextBox
$inputBox.Location = New-Object System.Drawing.Point(10, 520)
$inputBox.Size = New-Object System.Drawing.Size(1080, 25)
$inputBox.Font = New-Object System.Drawing.Font("Courier New", 10)
$form.Controls.Add($inputBox)

# Send Button
$sendButton = New-Object System.Windows.Forms.Button
$sendButton.Text = "Send"
$sendButton.Location = New-Object System.Drawing.Point(1100, 520)
$sendButton.Size = New-Object System.Drawing.Size(80, 25)
$form.Controls.Add($sendButton)

# Status Label
$statusLabel = New-Object System.Windows.Forms.Label
$statusLabel.Location = New-Object System.Drawing.Point(10, 555)
$statusLabel.Size = New-Object System.Drawing.Size(1170, 25)
$statusLabel.Text = "Status: Disconnected"
$statusLabel.BackColor = [System.Drawing.Color]::LightCoral
$statusLabel.Font = New-Object System.Drawing.Font("Courier New", 9, [System.Drawing.FontStyle]::Bold)
$form.Controls.Add($statusLabel)

# Command Buttons
$x = 10
$y = 590
$commands = @(
    @("pair_mode on", "▶ Pairing ON"),
    @("pair_mode off", "⏹ Pairing OFF"),
    @("sensors list", "📋 List"),
    @("config show", "⚙ Config"),
    @("status", "ℹ Status"),
    @("config reset", "🔄 Reset"),
    @("help", "? Help")
)

$cmdButtons = @()
foreach ($cmd in $commands) {
    $btn = New-Object System.Windows.Forms.Button
    $btn.Text = $cmd[1]
    $btn.Width = 130
    $btn.Height = 30
    $btn.Tag = $cmd[0]
    $btn.Location = New-Object System.Drawing.Point($x, $y)
    $btn.Font = New-Object System.Drawing.Font("Courier New", 9)
    $form.Controls.Add($btn)
    $cmdButtons += $btn
    
    $x += 135
    if ($x -gt 1000) {
        $x = 10
        $y += 35
    }
}

# Global variables
$serialPort = $null
$readerThread = $null
$connected = $false

function Add-OutputLine {
    param([string]$line)
    
    if ($line.Length -gt 0) {
        $outputBox.AppendText("$line`r`n")
        $outputBox.SelectionStart = $outputBox.Text.Length
        $outputBox.ScrollToCaret()
        [System.Windows.Forms.Application]::DoEvents()
        
        # Update status based on output
        if ($line -like "*SENSOR_ADDED*") {
            $statusLabel.Text = "Status: ✓ Sensor Added"
            $statusLabel.BackColor = [System.Drawing.Color]::LightGreen
            [System.Windows.Forms.MessageBox]::Show($line, "Sensor Added!", "OK", "Information") | Out-Null
        }
        elseif ($line -like "*HUB_STATUS_CHANGED*") {
            $statusLabel.Text = "Status: ✓ Hub Status Changed"
            $statusLabel.BackColor = [System.Drawing.Color]::LightYellow
        }
        elseif ($line -like "*PAIRING_MODE_ENABLED*") {
            $statusLabel.Text = "Status: 🔓 PAIRING MODE ENABLED"
            $statusLabel.BackColor = [System.Drawing.Color]::LightBlue
        }
        elseif ($line -like "*NORMAL_MODE_ENABLED*") {
            $statusLabel.Text = "Status: 🔒 NORMAL MODE (Polling Active)"
            $statusLabel.BackColor = [System.Drawing.Color]::LightGreen
        }
        elseif ($line -like "*POLLING_START*") {
            $statusLabel.Text = "Status: 🔄 Polling..."
            $statusLabel.BackColor = [System.Drawing.Color]::LightYellow
        }
    }
}

function Read-SerialBuffer {
    $output = ""
    try {
        if ($serialPort.IsOpen) {
            $timeout = [DateTime]::Now.AddSeconds(1)
            while ([DateTime]::Now -lt $timeout) {
                if ($serialPort.BytesToRead -gt 0) {
                    $output += $serialPort.ReadExisting()
                    Start-Sleep -Milliseconds 50
                }
            }
        }
    }
    catch { }
    return $output
}

function Send-Command {
    param([string]$cmd)
    
    if (-not $serialPort.IsOpen) {
        Add-OutputLine "✗ Serial port not open!"
        return
    }
    
    try {
        $inputBox.Text = ""
        $serialPort.WriteLine($cmd)
        $serialPort.BaseStream.Flush()
        
        Add-OutputLine ">> $cmd"
        Start-Sleep -Milliseconds 300
        
        $response = Read-SerialBuffer
        if ($response) {
            Add-OutputLine $response
        }
    }
    catch {
        Add-OutputLine "✗ Error sending command: $_"
    }
}

function Start-SerialReader {
    while ($connected -and $serialPort.IsOpen) {
        try {
            if ($serialPort.BytesToRead -gt 0) {
                $data = $serialPort.ReadExisting()
                $lines = $data -split "`n"
                
                foreach ($line in $lines) {
                    $line = $line.Trim()
                    if ($line.Length -gt 0) {
                        $form.Invoke([Action] {
                            Add-OutputLine $line
                        })
                    }
                }
            }
        }
        catch { }
        
        Start-Sleep -Milliseconds 100
    }
}

function Initialize-SerialPort {
    try {
        $script:serialPort = New-Object System.IO.Ports.SerialPort
        $serialPort.PortName = $port
        $serialPort.BaudRate = $baudRate
        $serialPort.Parity = [System.IO.Ports.Parity]::None
        $serialPort.DataBits = 8
        $serialPort.StopBits = [System.IO.Ports.StopBits]::One
        $serialPort.Handshake = [System.IO.Ports.Handshake]::None
        $serialPort.ReadTimeout = 500
        $serialPort.WriteTimeout = 500
        $serialPort.DtrEnable = $true
        $serialPort.RtsEnable = $true
        
        $serialPort.Open()
        $script:connected = $true
        
        $statusLabel.Text = "Status: ✓ Connected to $port @ $baudRate"
        $statusLabel.BackColor = [System.Drawing.Color]::LightGreen
        
        Add-OutputLine "✓ Connected to $port at $baudRate baud"
        Add-OutputLine "Reading device output..."
        
        # Read boot output
        Start-Sleep -Milliseconds 3000
        $bootOutput = Read-SerialBuffer
        if ($bootOutput) {
            Add-OutputLine $bootOutput
        }
        
        # Start reader thread
        $script:readerThread = [System.Threading.Thread]::new([System.Threading.ThreadStart]{
            Start-SerialReader
        })
        $readerThread.IsBackground = $true
        $readerThread.Start()
        
        return $true
    }
    catch {
        Add-OutputLine "✗ Connection failed: $_"
        $statusLabel.Text = "Status: ✗ Connection Failed"
        $statusLabel.BackColor = [System.Drawing.Color]::LightCoral
        return $false
    }
}

# Event Handlers
$sendButton.Add_Click({
    if ($inputBox.Text) {
        Send-Command $inputBox.Text
    }
})

$inputBox.Add_KeyDown({
    if ($_.KeyCode -eq "Enter") {
        Send-Command $inputBox.Text
        $_.Handled = $true
    }
})

foreach ($btn in $cmdButtons) {
    $btn.Add_Click({
        Send-Command $this.Tag
    })
}

$form.Add_Shown({
    if (Initialize-SerialPort) {
        $inputBox.Focus()
    }
})

$form.Add_FormClosing({
    $script:connected = $false
    
    if ($readerThread) {
        $readerThread.Join(1000)
    }
    
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
        $serialPort.Dispose()
    }
})

# Show Form
[void]$form.ShowDialog()