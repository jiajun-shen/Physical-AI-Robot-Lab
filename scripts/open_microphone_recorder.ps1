<#
.SYNOPSIS
    Opens the project-wide, user-controlled local microphone recorder.

.DESCRIPTION
    This small Windows PowerShell 5 GUI is the only interactive capture entry
    point used by the Physical-AI Robot Lab.  It never begins recording when
    the script opens: the user must explicitly click Start Recording.  The
    user can then click Stop Recording, or the capture stops automatically at
    60 seconds.  Stop is sent to ffmpeg through its redirected standard input
    as `q`, so the WAV is finalized before evidence metadata is written.

    Each click creates a new timestamped directory below OutputRoot containing
    microphone.wav, capture_metadata.json, and ffmpeg_capture.log.  The
    metadata is compatible with the Phase-7 local-ASR evidence contract, but
    this recorder performs no ASR and uses no network services.

.EXAMPLE
    powershell.exe -NoProfile -ExecutionPolicy Bypass -STA -File .\scripts\open_microphone_recorder.ps1

.EXAMPLE
    powershell.exe -NoProfile -ExecutionPolicy Bypass -STA -File .\scripts\open_microphone_recorder.ps1 `
      -OutputRoot C:\Users\Example\Documents\Codex\microphone-captures
#>
[CmdletBinding()]
param(
    # Capture sessions are placed in unique child directories below this root.
    # It must stay outside the source repository.
    [string]$OutputRoot = (Join-Path -Path ([Environment]::GetFolderPath([Environment+SpecialFolder]::MyDocuments)) -ChildPath 'Codex\microphone-captures'),

    # This ASCII DirectShow alternative name was verified for the current
    # Windows microphone.  Keeping the default ASCII-only also avoids Windows
    # PowerShell 5 code-page ambiguity caused by localized device display names.
    [string]$DeviceName = '@device_cm_{33D9A762-90C8-11D0-BD43-00A0C911CE86}\wave_{DAF55499-0211-45EE-9B20-EEDEDBDF227A}',

    [string]$FfmpegPath = 'C:\Program Files\JianyingPro\Apps\5.7.0.11527\ffmpeg.exe'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if ([Threading.Thread]::CurrentThread.ApartmentState -ne [Threading.ApartmentState]::STA) {
    throw 'This recorder needs an STA PowerShell session. Run it with powershell.exe -STA -File scripts\open_microphone_recorder.ps1.'
}

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

$script:maximumDurationSec = 60.0
$script:repositoryRoot = [IO.Directory]::GetParent([IO.Path]::GetFullPath($PSScriptRoot)).FullName
$script:recorder = $null
$script:recorderErrorHandler = $null
$script:recordingFolder = $null
$script:audioTemporaryPath = $null
$script:audioPath = $null
$script:metadataTemporaryPath = $null
$script:metadataPath = $null
$script:ffmpegLogPath = $null
$script:startedAtUnixNs = [Int64]0
$script:stopRequestedAtUtc = $null
$script:elapsedWatch = $null
$script:stopRequested = $false
$script:forcedTermination = $false
$script:closeAfterFinalization = $false
$script:finalizing = $false
$script:lastCompletedFolder = $null
$script:ffmpegLogLines = New-Object 'System.Collections.Generic.List[string]'

function Set-UiStatus {
    param(
        [Parameter(Mandatory = $true)][string]$Message,
        [System.Drawing.Color]$Color = [System.Drawing.Color]::DarkSlateGray
    )

    $script:statusLabel.Text = $Message
    $script:statusLabel.ForeColor = $Color
}

function Get-UnixNanoseconds {
    # This is an acquisition timestamp, not a hardware sample-clock claim.
    return ([Int64]([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()) * [Int64]1000000)
}

function Quote-WindowsArgument {
    param([Parameter(Mandatory = $true)][string]$Argument)

    # ProcessStartInfo.Arguments is a single command-line string on Windows
    # PowerShell 5/.NET Framework.  Apply the documented CommandLineToArgvW
    # escaping rules rather than relying on unquoted paths or device names.
    if ($Argument.Length -gt 0 -and $Argument -notmatch '[\s\"]') {
        return $Argument
    }

    $builder = New-Object System.Text.StringBuilder
    [void]$builder.Append([char]34)
    $backslashes = 0
    foreach ($character in $Argument.ToCharArray()) {
        if ($character -eq [char]92) {
            $backslashes++
            continue
        }
        if ($character -eq [char]34) {
            [void]$builder.Append([char]92, ($backslashes * 2 + 1))
            [void]$builder.Append([char]34)
            $backslashes = 0
            continue
        }
        if ($backslashes -gt 0) {
            [void]$builder.Append([char]92, $backslashes)
            $backslashes = 0
        }
        [void]$builder.Append($character)
    }
    if ($backslashes -gt 0) {
        [void]$builder.Append([char]92, ($backslashes * 2))
    }
    [void]$builder.Append([char]34)
    return $builder.ToString()
}

function Get-WaveFormat {
    param([Parameter(Mandatory = $true)][string]$Path)

    $format = $null
    $dataBytes = $null
    $stream = [IO.File]::Open($Path, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::Read)
    try {
        $reader = New-Object IO.BinaryReader($stream, [Text.Encoding]::ASCII, $false)
        try {
            $riff = [Text.Encoding]::ASCII.GetString($reader.ReadBytes(4))
            [void]$reader.ReadUInt32()
            $wave = [Text.Encoding]::ASCII.GetString($reader.ReadBytes(4))
            if ($riff -ne 'RIFF' -or $wave -ne 'WAVE') {
                throw 'ffmpeg did not produce a RIFF/WAVE file.'
            }

            while (($stream.Position + 8) -le $stream.Length) {
                $chunkId = [Text.Encoding]::ASCII.GetString($reader.ReadBytes(4))
                $chunkSize = [UInt32]$reader.ReadUInt32()
                if ([Int64]$chunkSize -gt ($stream.Length - $stream.Position)) {
                    throw 'The WAV contains a truncated chunk.'
                }
                if ($chunkId -eq 'fmt ') {
                    if ($chunkSize -lt 16) {
                        throw 'The WAV fmt chunk is too small.'
                    }
                    $format = [ordered]@{
                        audio_format = [Int32]$reader.ReadUInt16()
                        channels = [Int32]$reader.ReadUInt16()
                        sample_rate_hz = [Int32]$reader.ReadUInt32()
                        byte_rate = [Int32]$reader.ReadUInt32()
                        block_align = [Int32]$reader.ReadUInt16()
                        bits_per_sample = [Int32]$reader.ReadUInt16()
                    }
                    $remaining = [Int64]$chunkSize - 16
                    if ($remaining -gt 0) {
                        [void]$stream.Seek($remaining, [IO.SeekOrigin]::Current)
                    }
                } elseif ($chunkId -eq 'data') {
                    $dataBytes = [Int64]$chunkSize
                    [void]$stream.Seek([Int64]$chunkSize, [IO.SeekOrigin]::Current)
                } else {
                    [void]$stream.Seek([Int64]$chunkSize, [IO.SeekOrigin]::Current)
                }
                if (($chunkSize % 2) -eq 1 -and $stream.Position -lt $stream.Length) {
                    [void]$stream.Seek(1, [IO.SeekOrigin]::Current)
                }
            }
        } finally {
            $reader.Dispose()
        }
    } finally {
        $stream.Dispose()
    }

    if ($null -eq $format -or $null -eq $dataBytes) {
        throw 'The WAV lacks a fmt or data chunk.'
    }
    if ($format.audio_format -ne 1 -or $format.channels -ne 1 -or $format.sample_rate_hz -ne 16000 -or $format.bits_per_sample -ne 16 -or $format.block_align -ne 2) {
        throw 'The WAV is not mono 16 kHz signed-16-bit PCM audio.'
    }
    if ($dataBytes -le 0 -or ($dataBytes % $format.block_align) -ne 0) {
        throw 'The WAV data payload is invalid.'
    }

    $frames = [Int64]($dataBytes / $format.block_align)
    return [ordered]@{
        container = 'WAV'
        sample_rate_hz = $format.sample_rate_hz
        channels = $format.channels
        sample_width_bytes = [Int32]($format.bits_per_sample / 8)
        frames = $frames
        duration_sec = [double]$frames / [double]$format.sample_rate_hz
    }
}

function Resolve-OutputRoot {
    param([Parameter(Mandatory = $true)][string]$Candidate)

    if ([string]::IsNullOrWhiteSpace($Candidate)) {
        throw 'Choose an output folder before starting a recording.'
    }
    $resolved = [IO.Path]::GetFullPath($Candidate)
    $filesystemRoot = [IO.Path]::GetPathRoot($resolved)
    if ($resolved -eq $filesystemRoot) {
        throw 'The output folder cannot be a filesystem root.'
    }
    if (Test-Path -LiteralPath $resolved) {
        if (-not (Test-Path -LiteralPath $resolved -PathType Container)) {
            throw "The output path is not a folder: $resolved"
        }
    }

    $separators = [char[]]@([IO.Path]::DirectorySeparatorChar, [IO.Path]::AltDirectorySeparatorChar)
    $repository = [IO.Path]::GetFullPath($script:repositoryRoot).TrimEnd($separators)
    $output = $resolved.TrimEnd($separators)
    $repositoryPrefix = $repository + [IO.Path]::DirectorySeparatorChar
    if ($output.Equals($repository, [StringComparison]::OrdinalIgnoreCase) -or $output.StartsWith($repositoryPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'The output folder must be outside the source repository.'
    }
    return $resolved
}

function New-CaptureFolder {
    param([Parameter(Mandatory = $true)][string]$Root)

    [void][IO.Directory]::CreateDirectory($Root)
    $stamp = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ', [Globalization.CultureInfo]::InvariantCulture)
    $folder = Join-Path -Path $Root -ChildPath ("capture-$stamp")
    if (Test-Path -LiteralPath $folder) {
        $folder = Join-Path -Path $Root -ChildPath ("capture-$stamp-" + [Guid]::NewGuid().ToString('N'))
    }
    return [IO.Directory]::CreateDirectory($folder).FullName
}

function Write-FfmpegLog {
    if ([string]::IsNullOrWhiteSpace($script:ffmpegLogPath)) {
        return
    }
    $lines = @($script:ffmpegLogLines.ToArray())
    [IO.File]::WriteAllLines($script:ffmpegLogPath, [string[]]$lines, [Text.UTF8Encoding]::new($false))
}

function Reset-RecorderState {
    if ($null -ne $script:recorder) {
        # This is normally already exited when called from Finish-Recording.
        # It also covers a rare startup failure after Process.Start succeeds,
        # so an exception cannot leave an invisible microphone capture running.
        $isRunning = $false
        try { $isRunning = -not $script:recorder.HasExited } catch { }
        if ($isRunning) {
            try {
                $script:recorder.StandardInput.WriteLine('q')
                $script:recorder.StandardInput.Flush()
                [void]$script:recorder.WaitForExit(3000)
            } catch { }
            try {
                if (-not $script:recorder.HasExited) {
                    $script:recorder.Kill()
                    [void]$script:recorder.WaitForExit(3000)
                }
            } catch { }
        }
        try { $script:recorder.StandardInput.Close() } catch { }
        try {
            if ($null -ne $script:recorderErrorHandler) {
                $script:recorder.remove_ErrorDataReceived($script:recorderErrorHandler)
            }
        } catch { }
        try { $script:recorder.Dispose() } catch { }
    }
    $script:recorder = $null
    $script:recorderErrorHandler = $null
    $script:elapsedWatch = $null
    $script:stopRequestedAtUtc = $null
    $script:stopRequested = $false
    $script:forcedTermination = $false
}

function Finish-Recording {
    if ($script:finalizing -or $null -eq $script:recorder) {
        return
    }
    $script:finalizing = $true
    $script:recordingTimer.Stop()
    $endedAtUnixNs = Get-UnixNanoseconds
    $completed = $false
    $completionMessage = $null
    try {
        # Waiting after HasExited also lets asynchronous error callbacks drain.
        $script:recorder.WaitForExit()
        try { $script:recorder.CancelErrorRead() } catch { }
        Write-FfmpegLog

        if ($script:forcedTermination) {
            throw 'ffmpeg did not finalize after Stop Recording was requested; no microphone evidence was accepted.'
        }
        if ($script:recorder.ExitCode -ne 0) {
            throw "ffmpeg exited with code $($script:recorder.ExitCode). See $script:ffmpegLogPath"
        }
        if (-not (Test-Path -LiteralPath $script:audioTemporaryPath -PathType Leaf)) {
            throw 'ffmpeg exited successfully but did not create a temporary WAV.'
        }

        $audioFormat = Get-WaveFormat -Path $script:audioTemporaryPath
        if ($audioFormat.duration_sec -lt 0.5) {
            throw 'The recording is too short. Record at least half a second of speech before stopping.'
        }
        Move-Item -LiteralPath $script:audioTemporaryPath -Destination $script:audioPath -ErrorAction Stop
        $audioFile = Get-Item -LiteralPath $script:audioPath -ErrorAction Stop
        $metadata = [ordered]@{
            schema_version = 1
            capture_method = 'windows_directshow'
            input_kind = 'microphone'
            source = $script:sourceTextBox.Text.Trim()
            capture_backend = 'ffmpeg_directshow'
            audio_filename = 'microphone.wav'
            started_at_unix_ns = $script:startedAtUnixNs
            ended_at_unix_ns = $endedAtUnixNs
            duration_requested_sec = [double]$script:maximumDurationSec
            audio_sha256 = (Get-FileHash -LiteralPath $script:audioPath -Algorithm SHA256).Hash.ToLowerInvariant()
            audio_size_bytes = [Int64]$audioFile.Length
            audio_format = $audioFormat
        }
        $metadataJson = $metadata | ConvertTo-Json -Depth 6
        [IO.File]::WriteAllText($script:metadataTemporaryPath, $metadataJson + [Environment]::NewLine, [Text.UTF8Encoding]::new($false))
        Move-Item -LiteralPath $script:metadataTemporaryPath -Destination $script:metadataPath -ErrorAction Stop
        $script:lastCompletedFolder = $script:recordingFolder
        $script:lastOutputLabel.Text = "WAV: $($script:audioPath)`r`nMetadata: $($script:metadataPath)`r`nSHA-256: $($metadata.audio_sha256)"
        $script:openFolderButton.Enabled = $true
        $completionMessage = "Recording saved. WAV and capture metadata are ready in $($script:recordingFolder)"
        $completed = $true
    } catch {
        try { Write-FfmpegLog } catch { }
        $completionMessage = "Recording was not accepted: $($_.Exception.Message)"
    } finally {
        Reset-RecorderState
        $script:startButton.Enabled = $true
        $script:stopButton.Enabled = $false
        $script:outputRootTextBox.Enabled = $true
        $script:browseButton.Enabled = $true
        $script:sourceTextBox.Enabled = $true
        $script:ffmpegPathTextBox.Enabled = $true
        $script:elapsedLabel.Text = 'Elapsed: 00:00.0 / 01:00'
        $script:finalizing = $false
    }

    if ($completed) {
        Set-UiStatus -Message $completionMessage -Color ([Drawing.Color]::ForestGreen)
    } else {
        Set-UiStatus -Message $completionMessage -Color ([Drawing.Color]::Firebrick)
    }
    if ($script:closeAfterFinalization) {
        $script:closeAfterFinalization = $false
        $script:form.BeginInvoke([Action] { $script:form.Close() }) | Out-Null
    }
}

function Request-RecordingStop {
    param([Parameter(Mandatory = $true)][string]$Reason)

    if ($null -eq $script:recorder -or $script:stopRequested) {
        return
    }
    $script:stopRequested = $true
    $script:stopRequestedAtUtc = [DateTime]::UtcNow
    $script:stopButton.Enabled = $false
    Set-UiStatus -Message "$Reason Finalizing the WAV..." -Color ([Drawing.Color]::DarkOrange)
    try {
        if (-not $script:recorder.HasExited) {
            # Do not terminate ffmpeg abruptly: q lets it write the WAV header
            # with the final length before we hash and seal the artifact.
            $script:recorder.StandardInput.WriteLine('q')
            $script:recorder.StandardInput.Flush()
        }
    } catch {
        Set-UiStatus -Message "Could not request graceful ffmpeg stop: $($_.Exception.Message)" -Color ([Drawing.Color]::Firebrick)
    }
}

function Start-Recording {
    if ($null -ne $script:recorder) {
        Set-UiStatus -Message 'A recording is already active.' -Color ([Drawing.Color]::Firebrick)
        return
    }

    try {
        $resolvedOutputRoot = Resolve-OutputRoot -Candidate $script:outputRootTextBox.Text.Trim()
        $source = $script:sourceTextBox.Text.Trim()
        $ffmpeg = $script:ffmpegPathTextBox.Text.Trim()
        if ([string]::IsNullOrWhiteSpace($source)) {
            throw 'Enter a DirectShow microphone source before starting.'
        }
        if ($source -match '(?i)\.monitor$') {
            throw 'A physical microphone source is required; monitor sources are not accepted.'
        }
        if ([string]::IsNullOrWhiteSpace($ffmpeg) -or -not (Test-Path -LiteralPath $ffmpeg -PathType Leaf)) {
            throw "ffmpeg was not found: $ffmpeg"
        }

        $script:recordingFolder = New-CaptureFolder -Root $resolvedOutputRoot
        $script:audioTemporaryPath = Join-Path -Path $script:recordingFolder -ChildPath '.microphone.wav.partial'
        $script:audioPath = Join-Path -Path $script:recordingFolder -ChildPath 'microphone.wav'
        $script:metadataTemporaryPath = Join-Path -Path $script:recordingFolder -ChildPath '.capture_metadata.json.partial'
        $script:metadataPath = Join-Path -Path $script:recordingFolder -ChildPath 'capture_metadata.json'
        $script:ffmpegLogPath = Join-Path -Path $script:recordingFolder -ChildPath 'ffmpeg_capture.log'
        $script:ffmpegLogLines = New-Object 'System.Collections.Generic.List[string]'
        $script:startedAtUnixNs = Get-UnixNanoseconds
        $script:stopRequested = $false
        $script:forcedTermination = $false

        $arguments = @(
            '-hide_banner', '-loglevel', 'warning', '-y',
            '-f', 'dshow', '-i', ("audio=$source"),
            '-t', ([double]$script:maximumDurationSec).ToString('0.###', [Globalization.CultureInfo]::InvariantCulture),
            '-ac', '1', '-ar', '16000', '-acodec', 'pcm_s16le',
            '-f', 'wav', $script:audioTemporaryPath
        )
        $startInfo = New-Object Diagnostics.ProcessStartInfo
        $startInfo.FileName = $ffmpeg
        $startInfo.Arguments = (($arguments | ForEach-Object { Quote-WindowsArgument -Argument ([string]$_) }) -join ' ')
        $startInfo.UseShellExecute = $false
        $startInfo.CreateNoWindow = $true
        $startInfo.RedirectStandardInput = $true
        $startInfo.RedirectStandardError = $true
        $startInfo.RedirectStandardOutput = $false

        $script:recorder = New-Object Diagnostics.Process
        $script:recorder.StartInfo = $startInfo
        $script:recorderErrorHandler = [Diagnostics.DataReceivedEventHandler] {
            param($sender, $eventArgs)
            if ($null -ne $eventArgs.Data) {
                $script:ffmpegLogLines.Add([string]$eventArgs.Data)
                if ($script:ffmpegLogLines.Count -gt 400) {
                    $script:ffmpegLogLines.RemoveAt(0)
                }
            }
        }
        $script:recorder.add_ErrorDataReceived($script:recorderErrorHandler)
        if (-not $script:recorder.Start()) {
            throw 'Windows could not start ffmpeg.'
        }
        $script:recorder.BeginErrorReadLine()
        $script:elapsedWatch = [Diagnostics.Stopwatch]::StartNew()
        $script:recordingTimer.Start()

        $script:startButton.Enabled = $false
        $script:stopButton.Enabled = $true
        $script:outputRootTextBox.Enabled = $false
        $script:browseButton.Enabled = $false
        $script:sourceTextBox.Enabled = $false
        $script:ffmpegPathTextBox.Enabled = $false
        $script:lastOutputLabel.Text = "Recording to: $($script:recordingFolder)"
        Set-UiStatus -Message 'Recording is active. Speak now, then click Stop Recording.' -Color ([Drawing.Color]::Crimson)
    } catch {
        Reset-RecorderState
        $script:startButton.Enabled = $true
        $script:stopButton.Enabled = $false
        Set-UiStatus -Message "Could not start recording: $($_.Exception.Message)" -Color ([Drawing.Color]::Firebrick)
    }
}

# Build the window before registering actions.  No capture process is created
# here or anywhere else until the Start Recording button is clicked.
$script:form = New-Object Windows.Forms.Form
$script:form.Text = 'Physical AI Microphone Recorder'
$script:form.StartPosition = [Windows.Forms.FormStartPosition]::CenterScreen
$script:form.ClientSize = New-Object Drawing.Size(800, 435)
$script:form.MinimumSize = New-Object Drawing.Size(800, 435)
$script:form.MaximizeBox = $false
$script:form.FormBorderStyle = [Windows.Forms.FormBorderStyle]::FixedDialog

$instructionLabel = New-Object Windows.Forms.Label
$instructionLabel.AutoSize = $false
$instructionLabel.Location = New-Object Drawing.Point(18, 15)
$instructionLabel.Size = New-Object Drawing.Size(765, 46)
$instructionLabel.Text = "No audio is recorded until you click Start Recording. Capture stays local: this tool does not use a network or run speech recognition. Click Stop Recording when you finish; recording also stops after 60 seconds."
$instructionLabel.Font = New-Object Drawing.Font('Segoe UI', 9)
$script:form.Controls.Add($instructionLabel)

$outputLabel = New-Object Windows.Forms.Label
$outputLabel.AutoSize = $true
$outputLabel.Location = New-Object Drawing.Point(18, 77)
$outputLabel.Text = 'Save root (a new timestamped capture folder is created each time):'
$script:form.Controls.Add($outputLabel)

$script:outputRootTextBox = New-Object Windows.Forms.TextBox
$script:outputRootTextBox.Location = New-Object Drawing.Point(18, 98)
$script:outputRootTextBox.Size = New-Object Drawing.Size(652, 24)
$script:outputRootTextBox.Text = $OutputRoot
$script:form.Controls.Add($script:outputRootTextBox)

$script:browseButton = New-Object Windows.Forms.Button
$script:browseButton.Location = New-Object Drawing.Point(681, 96)
$script:browseButton.Size = New-Object Drawing.Size(101, 28)
$script:browseButton.Text = 'Browse...'
$script:form.Controls.Add($script:browseButton)

$sourceLabel = New-Object Windows.Forms.Label
$sourceLabel.AutoSize = $true
$sourceLabel.Location = New-Object Drawing.Point(18, 139)
$sourceLabel.Text = 'DirectShow microphone source:'
$script:form.Controls.Add($sourceLabel)

$script:sourceTextBox = New-Object Windows.Forms.TextBox
$script:sourceTextBox.Location = New-Object Drawing.Point(18, 160)
$script:sourceTextBox.Size = New-Object Drawing.Size(764, 24)
$script:sourceTextBox.Text = $DeviceName
$script:form.Controls.Add($script:sourceTextBox)

$ffmpegLabel = New-Object Windows.Forms.Label
$ffmpegLabel.AutoSize = $true
$ffmpegLabel.Location = New-Object Drawing.Point(18, 201)
$ffmpegLabel.Text = 'ffmpeg executable:'
$script:form.Controls.Add($ffmpegLabel)

$script:ffmpegPathTextBox = New-Object Windows.Forms.TextBox
$script:ffmpegPathTextBox.Location = New-Object Drawing.Point(18, 222)
$script:ffmpegPathTextBox.Size = New-Object Drawing.Size(764, 24)
$script:ffmpegPathTextBox.Text = $FfmpegPath
$script:form.Controls.Add($script:ffmpegPathTextBox)

$script:startButton = New-Object Windows.Forms.Button
$script:startButton.Location = New-Object Drawing.Point(18, 268)
$script:startButton.Size = New-Object Drawing.Size(174, 39)
$script:startButton.Text = 'Start Recording'
$script:startButton.Font = New-Object Drawing.Font('Segoe UI', 10, [Drawing.FontStyle]::Bold)
$script:form.Controls.Add($script:startButton)

$script:stopButton = New-Object Windows.Forms.Button
$script:stopButton.Location = New-Object Drawing.Point(204, 268)
$script:stopButton.Size = New-Object Drawing.Size(174, 39)
$script:stopButton.Text = 'Stop Recording'
$script:stopButton.Enabled = $false
$script:stopButton.Font = New-Object Drawing.Font('Segoe UI', 10, [Drawing.FontStyle]::Bold)
$script:form.Controls.Add($script:stopButton)

$script:openFolderButton = New-Object Windows.Forms.Button
$script:openFolderButton.Location = New-Object Drawing.Point(390, 268)
$script:openFolderButton.Size = New-Object Drawing.Size(152, 39)
$script:openFolderButton.Text = 'Open Saved Folder'
$script:openFolderButton.Enabled = $false
$script:form.Controls.Add($script:openFolderButton)

$script:elapsedLabel = New-Object Windows.Forms.Label
$script:elapsedLabel.AutoSize = $true
$script:elapsedLabel.Location = New-Object Drawing.Point(570, 278)
$script:elapsedLabel.Font = New-Object Drawing.Font('Consolas', 11, [Drawing.FontStyle]::Bold)
$script:elapsedLabel.Text = 'Elapsed: 00:00.0 / 01:00'
$script:form.Controls.Add($script:elapsedLabel)

$script:statusLabel = New-Object Windows.Forms.Label
$script:statusLabel.AutoSize = $false
$script:statusLabel.Location = New-Object Drawing.Point(18, 321)
$script:statusLabel.Size = New-Object Drawing.Size(764, 22)
$script:statusLabel.Font = New-Object Drawing.Font('Segoe UI', 9, [Drawing.FontStyle]::Bold)
$script:form.Controls.Add($script:statusLabel)

$script:lastOutputLabel = New-Object Windows.Forms.Label
$script:lastOutputLabel.AutoSize = $false
$script:lastOutputLabel.Location = New-Object Drawing.Point(18, 347)
$script:lastOutputLabel.Size = New-Object Drawing.Size(764, 66)
$script:lastOutputLabel.Font = New-Object Drawing.Font('Segoe UI', 8)
$script:form.Controls.Add($script:lastOutputLabel)

$script:recordingTimer = New-Object Windows.Forms.Timer
$script:recordingTimer.Interval = 100

$script:browseButton.Add_Click({
    $dialog = New-Object Windows.Forms.FolderBrowserDialog
    $dialog.Description = 'Choose an external folder for microphone capture sessions.'
    if (Test-Path -LiteralPath $script:outputRootTextBox.Text -PathType Container) {
        $dialog.SelectedPath = $script:outputRootTextBox.Text
    }
    try {
        if ($dialog.ShowDialog($script:form) -eq [Windows.Forms.DialogResult]::OK) {
            $script:outputRootTextBox.Text = $dialog.SelectedPath
        }
    } finally {
        $dialog.Dispose()
    }
})

$script:startButton.Add_Click({ Start-Recording })
$script:stopButton.Add_Click({ Request-RecordingStop -Reason 'Stop Recording was clicked.' })
$script:openFolderButton.Add_Click({
    if (-not [string]::IsNullOrWhiteSpace($script:lastCompletedFolder) -and (Test-Path -LiteralPath $script:lastCompletedFolder -PathType Container)) {
        Start-Process -FilePath 'explorer.exe' -ArgumentList @($script:lastCompletedFolder)
    }
})

$script:recordingTimer.Add_Tick({
    if ($null -eq $script:recorder -or $null -eq $script:elapsedWatch) {
        return
    }
    $elapsed = $script:elapsedWatch.Elapsed.TotalSeconds
    $script:elapsedLabel.Text = ('Elapsed: {0:mm\:ss\.f} / 01:00' -f $script:elapsedWatch.Elapsed)
    if ($script:recorder.HasExited) {
        Finish-Recording
        return
    }
    if (-not $script:stopRequested -and $elapsed -ge $script:maximumDurationSec) {
        Request-RecordingStop -Reason 'The 60-second maximum was reached.'
        return
    }
    if ($script:stopRequested -and $null -ne $script:stopRequestedAtUtc -and (([DateTime]::UtcNow - $script:stopRequestedAtUtc).TotalSeconds -ge 10.0)) {
        try {
            if (-not $script:recorder.HasExited) {
                # q is always attempted first. This fallback avoids leaving an
                # invisible capture process running if ffmpeg becomes wedged.
                $script:forcedTermination = $true
                $script:recorder.Kill()
            }
        } catch {
            Set-UiStatus -Message "ffmpeg did not exit after Stop Recording: $($_.Exception.Message)" -Color ([Drawing.Color]::Firebrick)
        }
    }
})

$script:form.Add_FormClosing({
    param($sender, $eventArgs)
    if ($null -ne $script:recorder -and -not $script:recorder.HasExited) {
        $eventArgs.Cancel = $true
        $script:closeAfterFinalization = $true
        Request-RecordingStop -Reason 'The window is closing.'
    }
})

Set-UiStatus -Message 'Ready. Choose Start Recording only when you are ready to speak.' -Color ([Drawing.Color]::DarkSlateGray)
[Windows.Forms.Application]::EnableVisualStyles()
[void][Windows.Forms.Application]::Run($script:form)
