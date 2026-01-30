#!/bin/bash

TARGET=stm32g474vetx
PROBE_ID="ATK 20190528"
ELF=bin/Kawashiro_Frame_G474.elf

usage() {
    echo "Usage: $0 {build|cmd|flash}"
    exit 1
}

[ $# -lt 1 ] && usage



case "$1" in
    build)
        cmake -B build  -DCMAKE_TOOLCHAIN_FILE=/home/hy/Kawashiro_Frame_G474_vsc/cmake/gcc-arm-none-eabi.cmake
        make -C build
        ;;
    cmd)
        pyocd commander -u "$PROBE_ID" -t "$TARGET"
        ;;
    flash)
        pyocd flash -u "$PROBE_ID" -t "$TARGET" "$ELF"
        ;;
    *)
        usage
        ;;
esac
