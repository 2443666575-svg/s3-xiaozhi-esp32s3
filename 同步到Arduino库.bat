@echo off
chcp 65001 >nul
title S3 Xiaozhi - Sync D drive source to Arduino library

rem ===================================================================
rem  用途: 把 D 盘资料库里的最新源码同步到 Arduino IDE 实际编译的库目录。
rem        改完代码双击本文件, 再去 Arduino IDE 点"上传"即可。
rem  只复制【源比目标新】的文件 (/XO), 不会覆盖库目录里更新的文件,
rem  也不会删除库目录里多出来的备份文件 (.bak_xxx / .backup_xxx)。
rem  项目资料唯一目录: D:\WorkBuddy知识库\S3小智
rem ===================================================================

set "SRC=%~dp0Ai智能\ai_vox-1.3.2\examples\esp32s3\ai_voice_display_lcd"
set "DST=%USERPROFILE%\Documents\Arduino\libraries\AI_VOX\ai_vox-1.3.2\examples\esp32s3\ai_voice_display_lcd"

echo.
echo  SOURCE : %SRC%
echo  TARGET : %DST%
echo.

if not exist "%SRC%\main.cpp" (
  echo  [ERROR] Source not found.
  echo          %SRC%
  pause
  exit /b 1
)
if not exist "%DST%" (
  echo  [ERROR] Arduino library folder not found.
  echo          %DST%
  pause
  exit /b 1
)

echo  Syncing newer files ...
robocopy "%SRC%" "%DST%" /E /XO /R:1 /W:1 /NFL /NDL /NP

if %ERRORLEVEL% GEQ 8 (
  echo.
  echo  [FAILED] robocopy exit code = %ERRORLEVEL%
) else (
  echo.
  echo  [DONE] Synced. Now click Upload in Arduino IDE.
)
echo.
pause
