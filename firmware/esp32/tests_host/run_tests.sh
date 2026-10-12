#!/usr/bin/env bash
# Compila y corre los tests de protocol.h en la computadora (no necesita ESP32).
#   ./run_tests.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SALIDA="$(mktemp -d)/test_protocol"

g++ -std=c++11 -Wall -Wextra -Werror -I"$DIR/../main" "$DIR/test_protocol.cpp" -o "$SALIDA"
"$SALIDA"
