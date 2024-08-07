add_library(tjpg INTERFACE)

target_sources(tjpg INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tjpg.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tjpgd.c
)

target_link_libraries(tjpg
    INTERFACE
	pico_stdlib
    freertos
)

target_include_directories(tjpg
    INTERFACE
    .
)
