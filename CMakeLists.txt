project(EthercatLichuanTCL57E)
cmake_minimum_required(VERSION 3.2)

find_package(EtherCAT REQUIRED)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

add_executable(input_timer
    main.cpp
)

target_link_libraries(input_timer PUBLIC EtherLab::ethercat)