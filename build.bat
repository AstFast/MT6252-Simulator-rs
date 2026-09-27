REM 一键构建：Unicorn 的 CMake 构建需要 ninja，而 cargo 的 [env] 无法改写 PATH（见 README）
@echo off
setlocal
REM 交付包里不写死 VS 安装路径：先认环境变量 VS_NINJA，否则到 PATH 里找 ninja.exe
if defined VS_NINJA goto haveninja
for %%i in (ninja.exe) do if not "%%~dp$PATH:i"=="" set "VS_NINJA=%%~dp$PATH:i%%~nxi"
:haveninja
if not defined VS_NINJA (
  echo 找不到 ninja.exe：请先 set VS_NINJA=到ninja.exe的全路径 再跑本脚本
  echo Visual Studio 自带一份，也可以用 winget/choco 装 ninja
  exit /b 1
)

if not exist target\debug mkdir target\debug
if not exist target\debug\ninja.exe copy "%VS_NINJA%" target\debug\ninja.exe >nul

cargo build %*
endlocal
