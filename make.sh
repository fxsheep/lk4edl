#!/usr/bin/env bash
set -eu
TARGET="msm8937-secondary"

# the stock bootloader needs this to retreive the appended FDT offset
GenerateKernelHeader() {
    C="tmp.c"
    BIN="./tmp"
    KERNEL_SIZE="$1"
    HEADER_OUT="$2"

    # generate C file
    echo "#include <unistd.h>" > "$C"
    echo "#include <stdint.h>" >> "$C"
    echo "int main(void){int i;uint32_t n;" >> "$C"
    echo "n=0xea00000a; write(1, &n, sizeof(n));" >> "$C"   # b 0x30
    echo "for(i=0;i<8;i++)" >> "$C"
    echo "{n=0xe1a00000; write(1, &n, sizeof(n));}" >> "$C" # NOP
    echo "n=0x016f2818; write(1, &n, sizeof(n));" >> "$C"   # Magic numbers to help the loader
    echo "n=0x00000000; write(1, &n, sizeof(n));" >> "$C"   # absolute load/run zImage address
    echo "n=$KERNEL_SIZE; write(1, &n, sizeof(n));" >> "$C" # zImage end address
    echo "return 0;" >> "$C"
    echo "}" >> "$C"

    # compile C file
    gcc -Wall -Wextra -Wshadow -Werror "$C" -o "$BIN"

    # write header
    "$BIN" >> "$HEADER_OUT"
}

rm -rf "build-$TARGET"

make -j6 TOOLCHAIN_PREFIX=arm-none-eabi- "$TARGET"

cd "build-$TARGET"

LK_BINARY="lk.bin"
LK_BINARY_FINAL="lk_final.bin"
LK_SIZE="$(( 0x30 + $(stat -L -c %s $LK_BINARY) ))"

rm -f "$LK_BINARY_FINAL"
GenerateKernelHeader "$LK_SIZE" "$LK_BINARY_FINAL"
cat "$LK_BINARY" >> "$LK_BINARY_FINAL"

../scripts/mkbootimg \
    --kernel=$LK_BINARY_FINAL \
    --ramdisk=/dev/null \
    --dt=../../dt.img \
    --base=0x80000000 \
    --output=boot.img \
    --cmdline="lk2"
