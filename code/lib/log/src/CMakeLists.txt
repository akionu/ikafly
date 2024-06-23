
add_library(log INTERFACE)

target_sources(log INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/log.cpp
)

target_link_libraries(log
    INTERFACE
    pico_stdlib
	pico_flash
	hardware_flash
	freertos
)

target_include_directories(log INTERFACE
    .
)
