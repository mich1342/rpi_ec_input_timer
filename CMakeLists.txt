project(EthercatLichuanTCL57E)
cmake_minimum_required(VERSION 3.2)

find_package(EtherCAT REQUIRED)

add_executable(input_timer
    main.cpp
)

target_link_libraries(input_timer PUBLIC EtherLab::ethercat)