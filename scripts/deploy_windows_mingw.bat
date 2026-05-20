@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy_windows_mingw.ps1" %*
