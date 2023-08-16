#!/bin/bash

declare -A COMPILER=( [x86_64]=/usr/bin/gcc 
                      [aarch64]=/usr/bin/aarch64-linux-gnu-gcc 
                      [armv7l]=/usr/bin/arm-linux-gnueabi-gcc )

echo "ARCH: ${ARCH}"

for ARCH in x86_64 
do
    echo "-I- Building ${ARCH}"
    mkdir -p build/${ARCH}
    export CXX=g++-9
    LIB_VER=4.14.0 HAILORT_ROOT=${HAILORT_ROOT} cmake -H. -Bbuild/${ARCH} -DARCH=${ARCH} -DCMAKE_C_COMPILER=${COMPILER[${ARCH}]}
    cmake --build build/${ARCH}
done
if [[ -f "hailort.log" ]]; then
    rm hailort.log
fi
