@echo off
chcp 65001 >nul
setlocal
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist build mkdir build
cl /nologo /utf-8 /W3 /D_CRT_SECURE_NO_WARNINGS /I stubs /I "..\..\407_gps初始化失败 - 副本\Core\Inc" /I "..\..\407_gps初始化失败 - 副本\Core\Src" test_firmware.c /Fo:build\test_firmware.obj /Fe:build\test_firmware.exe
if errorlevel 1 exit /b 1
build\test_firmware.exe
exit /b %errorlevel%
