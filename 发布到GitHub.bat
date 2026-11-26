@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo.

set /p TOKEN=请输入 GitHub Personal Access Token (勾选 repo 权限): 
if "%TOKEN%"=="" (echo 未输入 Token，已退出。 & exit /b 1)

echo.
echo [1/6] 用 Token 自动创建远端仓库（网页端报 Something went wrong 时走这条）...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0create_repo.ps1" -Token "%TOKEN%"
if errorlevel 1 (echo 仓库创建失败，请检查 Token 的 repo 权限与网络。 & exit /b 1)

echo.
echo [2/6] 初始化 git ...
if not exist .git git init
git config user.name "2443666575-svg"
git config user.email "2443666575-svg@users.noreply.github.com"

echo [3/6] 添加文件 ...
git add -A

echo [4/6] 提交 ...
git commit -m "Initial commit: S3 Xiaozhi ESP32-S3 (sanitized)" >nul 2>&1

echo [5/6] 设置分支与远端 ...
git branch -M main
git remote remove origin >nul 2>&1
git remote add origin https://github.com/2443666575-svg/s3-xiaozhi-esp32s3.git

echo [6/6] 推送 ...
git -c http.extraHeader="AUTHORIZATION: bearer %TOKEN%" push -u origin main
if errorlevel 1 (
  echo.
  echo 推送失败。请确认 Token 已勾选 repo 权限、网络可访问 github.com。
  exit /b 1
)

echo.
echo 发布完成: https://github.com/2443666575-svg/s3-xiaozhi-esp32s3
echo 提醒: 请到 GitHub Settings ^> Developer settings 撤销此 Token。
echo       如之前泄露过智谱 API Key，请到智谱开放平台重置。
pause
