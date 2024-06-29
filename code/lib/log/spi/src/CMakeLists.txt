add_library(spiffs INTERFACE)

target_sources(spiffs INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/spiffs_cache.c
    ${CMAKE_CURRENT_LIST_DIR}/spiffs_check.c
    ${CMAKE_CURRENT_LIST_DIR}/spiffs_gc.c
    ${CMAKE_CURRENT_LIST_DIR}/spiffs_hydrogen.c
    ${CMAKE_CURRENT_LIST_DIR}/spiffs_nucleus.c
)

target_link_libraries(spiffs
    INTERFACE
	pico_stdlib
	pico_flash
	hardware_flash
    freertos
)

target_include_directories(spiffs
    INTERFACE
    .
)
