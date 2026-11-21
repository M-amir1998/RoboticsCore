@echo off
rem Loads the newest Visual Studio (or Build Tools) x64 developer environment so that
rem cl.exe, Ninja, CMake, clang-format and clang-tidy shipped with VS are on PATH.
rem
rem   call scripts\setup\vs_env.cmd
rem   cmake --preset debug

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo vswhere.exe not found; install Visual Studio 2022+ or Build Tools with the C++ workload. 1>&2
  exit /b 1
)

for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
  set "VS_INSTALL=%%i"
)

if not defined VS_INSTALL (
  echo No Visual Studio installation with the C++ x64 toolset was found. 1>&2
  exit /b 1
)

call "%VS_INSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1

rem Make the LLVM tools bundled with VS (clang++, clang-format, clang-tidy) visible.
if exist "%VS_INSTALL%\VC\Tools\Llvm\x64\bin" set "PATH=%VS_INSTALL%\VC\Tools\Llvm\x64\bin;%PATH%"
exit /b 0
