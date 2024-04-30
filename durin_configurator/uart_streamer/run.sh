#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

# run
cd "$SCRIPT_DIR/../../"
source .venv/bin/activate
python durin_configurator/uart_streamer/uart_streamer.py /dev/ttyS7