#!/usr/bin/env bash
set -e

echo "[ffw-sh5] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
    cmake \
    build-essential \
    libglfw3-dev \
    libglew-dev \
    libbullet-dev \
    libtinyxml2-dev \
    libgl1-mesa-dev \
    git

echo "[ffw-sh5] Initializing submodules..."
git -C "$(git rev-parse --show-toplevel)" submodule update --init --recursive

echo "[ffw-sh5] Done. Run:"
echo "  mkdir build && cd build && cmake .. && make -j\$(nproc)"
