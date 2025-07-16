@echo off
pip install pdoc
cd /d "%~dp0.\code"
python -m pdoc evpylib.py -o ../docs
pause