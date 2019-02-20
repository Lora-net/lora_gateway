
add_executable(util_pkt_logger "")
target_sources(util_pkt_logger
	PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/util_pkt_logger.c
        ${CMAKE_CURRENT_LIST_DIR}/src/parson.c
)

target_include_directories(util_pkt_logger
	PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/inc
)

target_link_libraries(util_pkt_logger
	PUBLIC
		loragw
)

set_target_properties(util_pkt_logger PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
        TARGETS util_pkt_logger
  	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
