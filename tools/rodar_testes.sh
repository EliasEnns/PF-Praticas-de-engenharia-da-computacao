#!/usr/bin/env bash
# Roda os dois testes do enlace. Linux, macOS ou WSL.
set -e
cd "$(dirname "$0")/.."
echo "=== compilando o teste de integracao ==="
g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter \
    -I tools/stub -I baja_espnow_tx -I baja_espnow_rx \
    tools/test_bajalink.cpp -o /tmp/test_bajalink
/tmp/test_bajalink
echo
echo "=== contrato firmware <-> dashboard ==="
python3 tools/test_formato.py
