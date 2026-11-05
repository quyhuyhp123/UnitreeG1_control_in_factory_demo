@echo off
powershell.exe -NoProfile -File "D:\unitree_g1\scripts\run_sim.ps1" %*
exit /b %errorlevel%
