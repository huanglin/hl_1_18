#!/bin/bash
KERNEL_DIR=.
IMAGE=${KERNEL_DIR}/vmlinux
OBJDUMP=${KERNEL_DIR}/../toolchain/arm-eabi-4.4.0/bin/arm-eabi-objdump
echo "Geting ${IMAGE} assambler..."
${OBJDUMP} -d -S ${IMAGE} > linux.S

