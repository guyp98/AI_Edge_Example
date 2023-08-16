cmake_minimum_required(VERSION 3.11)
project(raw_async_cpp)

set(COMPILE_OPTIONS -Wall -Wextra -Wconversion -O3 -DNDEBUG -Wno-ignored-qualifiers -Wno-conversion -Wno-unused-but-set-parameter -Wno-extra -Wno-reorder -Wno-unused-local-typedefs)

set(BASE_DIR /path/to/yolov8/example/folder)

find_package(Threads)
find_package(HailoRT REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "Found OpenCV: " ${OpenCV_INCLUDE_DIRS})

file(GLOB SOURCES
./*.cpp
)

link_libraries(stdc++fs)
add_executable(${PROJECT_NAME} ${SOURCES})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("./common")
target_compile_options(${PROJECT_NAME} PRIVATE ${COMPILE_OPTIONS})
#target_link_libraries(${PROJECT_NAME} Threads::Threads HailoRT::libhailort)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} /usr/lib/libhailort.so)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
