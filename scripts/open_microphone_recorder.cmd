@echo off
setlocal
start "" powershell.exe -NoProfile -ExecutionPolicy Bypass -STA -File "%~dp0open_microphone_recorder.ps1"
