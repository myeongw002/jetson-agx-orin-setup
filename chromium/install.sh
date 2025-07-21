#!/usr/bin/env bash
set -euo pipefail

# 1. 패키지 목록 갱신
echo "1/5: Updating package lists..."
sudo apt update

# 2. Flatpak 설치
echo "2/5: Installing Flatpak..."
sudo apt install -y flatpak

# 3. Flathub 저장소 추가
echo "3/5: Adding Flathub repository..."
sudo flatpak remote-add --if-not-exists flathub https://dl.flathub.org/repo/flathub.flatpakrepo

# 4. Chromium 설치
echo "4/5: Installing Chromium from Flathub..."
sudo flatpak install -y flathub org.chromium.Chromium

# 5. Chromium 실행
echo "5/5: Launching Chromium..."
flatpak run org.chromium.Chromium

exit 0

