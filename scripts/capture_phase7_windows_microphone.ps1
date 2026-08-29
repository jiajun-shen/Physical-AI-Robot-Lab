<#
Capture one bounded, consented Phase-7 microphone WAV through Windows
DirectShow/ffmpeg.  It produces the same evidence metadata schema as the WSLg
PulseAudio capture script and intentionally accepts no typed transcript.

Example:
  .\capture_phase7_windows_microphone.ps1 `
    -OutputDirectory C:\Temp\physical-ai-p7\zh `
    -DurationSec 7 -RecordingConsentToken I_CONSENT_TO_MICROPHONE_RECORDING
#>
param(
    [string]$OutputDirectory,

    [double]$DurationSec,

    # Use the DirectShow alternative name rather than a localized display name:
    # Windows PowerShell 5 reads UTF-8 scripts through the active code page.
    [string]$DeviceName = '@device_cm_{33D9A762-90C8-11D0-BD43-00A0C911CE86}\wave_{DAF55499-0211-45EE-9B20-EEDEDBDF227A}',

    [string]$FfmpegPath = 'C:\Program Files\JianyingPro\Apps\5.7.0.11527\ffmpeg.exe',

    [string]$RecordingConsentToken
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Fail([string]$Message) {
    throw "ERROR: $Message"
}

function Get-UnixNanoseconds {
    # Windows' wall clock has finer-than-millisecond bookkeeping on most
    # installations, but this is intentionally an acquisition timestamp, not
    # a claim of hardware-sample-clock precision.
    return [Int64]([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()) * 1000000
}

function Get-WaveFormat([string]$Path) {
    $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::Read)
    try {
        $reader = [System.IO.BinaryReader]::new($stream, [System.Text.Encoding]::ASCII, $false)
        try {
            $riff = [System.Text.Encoding]::ASCII.GetString($reader.ReadBytes(4))
            [void]$reader.ReadUInt32()
            $wave = [System.Text.Encoding]::ASCII.GetString($reader.ReadBytes(4))
            if ($riff -ne 'RIFF' -or $wave -ne 'WAVE') {
                Fail 'ffmpeg did not produce a RIFF/WAVE file'
            }
            $format = $null
            $dataBytes = $null
            while ($stream.Position + 8 -le $stream.Length) {
                $chunkId = [System.Text.Encoding]::ASCII.GetString($reader.ReadBytes(4))
                $chunkSize = [UInt32]$reader.ReadUInt32()
                if ($chunkSize -gt ($stream.Length - $stream.Position)) {
                    Fail 'WAV contains a truncated chunk'
                }
                if ($chunkId -eq 'fmt ') {
                    if ($chunkSize -lt 16) { Fail 'WAV fmt chunk is too small' }
                    $format = [ordered]@{
                        audio_format = [Int32]$reader.ReadUInt16()
                        channels = [Int32]$reader.ReadUInt16()
                        sample_rate_hz = [Int32]$reader.ReadUInt32()
                        byte_rate = [Int32]$reader.ReadUInt32()
                        block_align = [Int32]$reader.ReadUInt16()
                        bits_per_sample = [Int32]$reader.ReadUInt16()
                    }
                    $remaining = [Int64]$chunkSize - 16
                    if ($remaining -gt 0) { [void]$stream.Seek($remaining, [System.IO.SeekOrigin]::Current) }
                } elseif ($chunkId -eq 'data') {
                    $dataBytes = [Int64]$chunkSize
                    [void]$stream.Seek([Int64]$chunkSize, [System.IO.SeekOrigin]::Current)
                } else {
                    [void]$stream.Seek([Int64]$chunkSize, [System.IO.SeekOrigin]::Current)
                }
                if (($chunkSize % 2) -eq 1 -and $stream.Position -lt $stream.Length) {
                    [void]$stream.Seek(1, [System.IO.SeekOrigin]::Current)
                }
            }
        } finally {
            $reader.Dispose()
        }
    } finally {
        $stream.Dispose()
    }
    if ($null -eq $format -or $null -eq $dataBytes) { Fail 'WAV lacks fmt or data chunk' }
    if ($format.audio_format -ne 1 -or $format.channels -ne 1 -or $format.sample_rate_hz -ne 16000 -or $format.bits_per_sample -ne 16 -or $format.block_align -ne 2) {
        Fail 'WAV is not mono 16 kHz PCM signed-16-bit audio'
    }
    if ($dataBytes -le 0 -or ($dataBytes % $format.block_align) -ne 0) { Fail 'WAV data payload is invalid' }
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

if ($RecordingConsentToken -ne 'I_CONSENT_TO_MICROPHONE_RECORDING') {
    Fail 'microphone capture requires explicit RecordingConsentToken'
}
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    Fail 'microphone capture requires OutputDirectory'
}
if ($DurationSec -lt 1.0 -or $DurationSec -gt 60.0) {
    Fail 'DurationSec must be between 1 and 60 seconds'
}
if ([string]::IsNullOrWhiteSpace($DeviceName) -or $DeviceName -match '(?i)\.monitor$') {
    Fail 'a physical DirectShow microphone device is required'
}
if (-not (Test-Path -LiteralPath $FfmpegPath -PathType Leaf)) {
    Fail "ffmpeg executable was not found: $FfmpegPath"
}

$scriptDirectory = Split-Path -Path $PSScriptRoot -Parent
$repositoryRoot = [System.IO.Path]::GetFullPath($scriptDirectory)
$resolvedOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
$repositoryPrefix = $repositoryRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
if ($resolvedOutput.StartsWith($repositoryPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    Fail 'capture output directory must not be inside repository'
}
if ($resolvedOutput -eq [System.IO.Path]::GetPathRoot($resolvedOutput)) {
    Fail 'capture output directory must not be a filesystem root'
}
if (Test-Path -LiteralPath $resolvedOutput) {
    if (-not (Test-Path -LiteralPath $resolvedOutput -PathType Container)) { Fail 'capture output path is not a directory' }
    if ([System.IO.Directory]::EnumerateFileSystemEntries($resolvedOutput).GetEnumerator().MoveNext()) {
        Fail 'capture output directory must be new or empty'
    }
} else {
    [void][System.IO.Directory]::CreateDirectory($resolvedOutput)
}

$audioTemporary = Join-Path -Path $resolvedOutput -ChildPath '.microphone.wav.partial'
$audioPath = Join-Path -Path $resolvedOutput -ChildPath 'microphone.wav'
$metadataTemporary = Join-Path -Path $resolvedOutput -ChildPath '.capture_metadata.json.partial'
$metadataPath = Join-Path -Path $resolvedOutput -ChildPath 'capture_metadata.json'
$ffmpegLog = Join-Path -Path $resolvedOutput -ChildPath 'ffmpeg_capture.log'
$startedAtUnixNs = Get-UnixNanoseconds

$ffmpegArguments = @(
    '-hide_banner', '-nostdin', '-y',
    '-f', 'dshow',
    '-i', "audio=$DeviceName",
    '-t', $DurationSec.ToString('0.###', [System.Globalization.CultureInfo]::InvariantCulture),
    '-ac', '1', '-ar', '16000', '-acodec', 'pcm_s16le',
    '-f', 'wav',
    $audioTemporary
)
$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
try {
    & $FfmpegPath @ffmpegArguments 2> $ffmpegLog
    $ffmpegExitCode = $LASTEXITCODE
} finally {
    $ErrorActionPreference = $previousErrorActionPreference
}
$endedAtUnixNs = Get-UnixNanoseconds
if ($ffmpegExitCode -ne 0) {
    Fail "ffmpeg DirectShow capture failed with exit code $ffmpegExitCode; inspect $ffmpegLog"
}
if (-not (Test-Path -LiteralPath $audioTemporary -PathType Leaf)) {
    Fail 'ffmpeg exited successfully but did not create WAV output'
}

$audioFormat = Get-WaveFormat -Path $audioTemporary
Move-Item -LiteralPath $audioTemporary -Destination $audioPath -ErrorAction Stop
$audioFile = Get-Item -LiteralPath $audioPath -ErrorAction Stop
$metadata = [ordered]@{
    schema_version = 1
    capture_method = 'windows_directshow'
    input_kind = 'microphone'
    source = $DeviceName
    capture_backend = 'ffmpeg_directshow'
    audio_filename = 'microphone.wav'
    started_at_unix_ns = $startedAtUnixNs
    ended_at_unix_ns = $endedAtUnixNs
    duration_requested_sec = $DurationSec
    audio_sha256 = (Get-FileHash -LiteralPath $audioPath -Algorithm SHA256).Hash.ToLowerInvariant()
    audio_size_bytes = [Int64]$audioFile.Length
    audio_format = $audioFormat
}
$metadataJson = $metadata | ConvertTo-Json -Depth 6
[System.IO.File]::WriteAllText($metadataTemporary, $metadataJson + [Environment]::NewLine, [System.Text.UTF8Encoding]::new($false))
Move-Item -LiteralPath $metadataTemporary -Destination $metadataPath -ErrorAction Stop

Write-Output 'P7_MICROPHONE_CAPTURE=PASS'
Write-Output "AUDIO=$audioPath"
Write-Output "METADATA=$metadataPath"
Write-Output "SHA256=$($metadata.audio_sha256)"
