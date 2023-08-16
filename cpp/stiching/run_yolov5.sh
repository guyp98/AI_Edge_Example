rm -rf build 
export HAILORT_ROOT=/usr/include/hailo
export LD_LIBRARY_PATH=/usr/lib/hailortcli
./build.sh
./build/x86_64/raw_async_cpp out2.png out1.png


