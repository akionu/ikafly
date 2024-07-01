add_subdirectory(lis3mdl)
add_subdirectory(lsm6dso)
add_subdirectory(MadgwickAHRS)

add_library(imu INTERFACE)

target_sources(imu INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/imu.cpp
	${CMAKE_CURRENT_LIST_DIR}/MadgwickAHRS/MadgwickAHRS.c
	#	${CMAKE_CURRENT_LIST_DIR}/lis3mdl/src/lis3mdl.c
	#	${CMAKE_CURRENT_LIST_DIR}/lis3mdl/src/lis3mdl-pid/lis3mdl_reg.c
	#	${CMAKE_CURRENT_LIST_DIR}/lsm6dso/src/lsm6dso.c
	#	${CMAKE_CURRENT_LIST_DIR}/lsm6dso/src/lsm6dso-pid/lsm6dso_reg.c
)

target_link_libraries(imu
    INTERFACE
    pico_stdlib
    hardware_i2c
	MadgwickAHRS
	freertos
	lis3mdl
	lsm6dso
)

target_include_directories(imu
    INTERFACE
    .
)
