@echo off
rem FW57J-B Flight Simulator - double-click to play (needs Python)
cd /d "%~dp0"
python tools\serve.py 8760 --open
pause
