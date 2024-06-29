add_library(littlefs INTERFACE)

target_sources(littlefs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/littlefs.cpp
    ${CMAKE_CURRENT_LIST_DIR}/littlefs/lfs.c
    ${CMAKE_CURRENT_LIST_DIR}/littlefs/lfs_util.c
)

target_link_libraries(littlefs
    INTERFACE
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
)

target_include_directories(littlefs
    INTERFACE
    .
    ./littlefs/
)
