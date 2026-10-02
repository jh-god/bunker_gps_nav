#!/usr/bin/env bash
set -e
BUNKER_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BUNKER_ARCHIVE=${1:-/tmp/bunker_gps_navigation-source.tar.gz}
tar --exclude='./build' --exclude='./install' --exclude='./log' \
    --exclude='./.desktop_deps' --exclude='./.git' --exclude='__pycache__' \
    -czf "$BUNKER_ARCHIVE" -C "$BUNKER_ROOT" .
printf 'Source archive: %s\n' "$BUNKER_ARCHIVE"
