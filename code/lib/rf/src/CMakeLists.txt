
add_library(rf INTERFACE)

target_sources(rf INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/rf.cpp
)

target_link_libraries(rf
        INTERFACE
        pico_stdlib
)
