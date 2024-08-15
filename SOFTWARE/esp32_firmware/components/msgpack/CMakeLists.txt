# See https://github.com/espressif/esp-idf/tree/master/examples/build_system/cmake/import_lib
idf_component_register()

include(ExternalProject)

set(MSGPACK_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/msgpack_install)

externalproject_add(msgpack_proj
    URL https://github.com/msgpack/msgpack-c/archive/refs/tags/c-6.0.2.zip

    CMAKE_ARGS
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>

    USES_TERMINAL_DOWNLOAD TRUE
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE

    INSTALL_DIR ${MSGPACK_INSTALL_DIR}
    BUILD_BYPRODUCTS "${MSGPACK_INSTALL_DIR}/lib/libmsgpack-c.a"
)

add_prebuilt_library(msgpack_lib "${MSGPACK_INSTALL_DIR}/lib/libmsgpack-c.a")
target_include_directories(msgpack_lib INTERFACE "${MSGPACK_INSTALL_DIR}/include")
add_dependencies(msgpack_lib msgpack_proj)

target_link_libraries(${COMPONENT_LIB} INTERFACE msgpack_lib)
