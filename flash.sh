#!/bin/bash

TARGET=stm32g474vetx
PROBE_ID="ATK 20190528"
ELF=build/Kawashiro_Frame_G474.elf

usage() {
    echo "Usage: $0 {cmd|reset|flash|all}"
    exit 1
}

[ $# -lt 1 ] && usage

case "$1" in
    cmd)
        pyocd commander -u "$PROBE_ID" -t "$TARGET"
        ;;
    reset)
        pyocd reset -u "$PROBE_ID" -t "$TARGET"
        ;;
    flash)
        pyocd flash -u "$PROBE_ID" -t "$TARGET" "$ELF"
        ;;
    all)
        pyocd flash -u "$PROBE_ID" -t "$TARGET" "$ELF" || exit 1
        pyocd reset -u "$PROBE_ID" -t "$TARGET"
        ;;
    *)
        usage
        ;;
esac