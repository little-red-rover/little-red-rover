set(pb_csrcs
    "${CMAKE_CURRENT_LIST_DIR}/messages.pb.c")
 
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    add_custom_command(
      OUTPUT ${pb_csrcs}
      COMMAND ${COMPONENT_DIR}/../nanopb/generator-bin/protoc -I ${COMPONENT_DIR} --nanopb_out=${CMAKE_CURRENT_LIST_DIR} messages.proto 
      DEPENDS ${COMPONENT_DIR}/messages.proto
      VERBATIM)
endif()

idf_component_register(SRCS  "socket_mgr.c" "pb_utils.c" "${pb_csrcs}"
                       INCLUDE_DIRS include
                       REQUIRES nvs_flash nanopb)
