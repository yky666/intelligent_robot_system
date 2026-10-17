@echo off
cd /d "%~dp0"
xelatex -interaction=nonstopmode -halt-on-error main.tex
if errorlevel 1 exit /b 1
xelatex -interaction=nonstopmode -halt-on-error main.tex
