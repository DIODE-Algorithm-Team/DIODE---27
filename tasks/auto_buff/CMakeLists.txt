cmake_minimum_required(VERSION 3.16)
set(OpenVINO_DIR "/home/gzy/openvino_env/lib/python3.12/site-packages/openvino/cmake")

find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

add_library(auto_buff OBJECT 
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

target_link_libraries(auto_buff auto_aim openvino::runtime ${CERES_LIBRARIES})