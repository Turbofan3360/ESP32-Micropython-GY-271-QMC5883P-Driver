add_library(usermod_qmc5883p INTERFACE)

target_sources(usermod_qmc5883p INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/qmc5883p.c
)

target_include_directories(usermod_qmc5883p INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_qmc5883p)