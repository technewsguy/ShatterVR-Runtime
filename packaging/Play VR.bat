@echo off
cd /d "%~dp0"
if not exist game.apk (
  echo Put your own APK in this folder and name it game.apk
  pause
  exit /b 1
)
sh_play.exe game.apk --vr
if errorlevel 1 pause
