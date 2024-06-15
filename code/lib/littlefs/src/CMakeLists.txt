add_library(littlefs INTERFACE)

target_sources(littlefs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/littlefs.cpp
)

target_link_libraries(littlefs
    INTERFACE
    pico_stdlib
	hardware_flash
	freertos
)

target_include_directories(littlefs INTERFACE
	../../freertos/*
)

