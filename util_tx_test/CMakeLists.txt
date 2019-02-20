
add_executable(util_tx_test "")
target_sources(util_tx_test
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/util_tx_test.c
)

target_link_libraries(util_tx_test
	PUBLIC
		loragw
)

set_target_properties(util_tx_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
        TARGETS util_tx_test
  	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
