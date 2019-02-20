
add_executable(util_lbt_test "")
target_sources(util_lbt_test
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/util_lbt_test.c
)

target_link_libraries(util_lbt_test
	PUBLIC
		loragw
)

set_target_properties(util_lbt_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
        TARGETS util_lbt_test
  	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
