rm -rf build 
#export HAILORT_ROOT=/usr/include/hailo
#export LD_LIBRARY_PATH=/usr/lib/hailortcli2
./build.sh

./build/x86_64/vstream_re_id_example -hef=yolov8s.hef -reid=repvgg_a0_person_reid_2048_16b.hef -num=1


